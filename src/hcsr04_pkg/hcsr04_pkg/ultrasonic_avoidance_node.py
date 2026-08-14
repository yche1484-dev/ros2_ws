#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray, Int32


class UltrasonicAvoidanceNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_avoidance_node')

        # =====================================================
        # ROS PARAMETERS
        # =====================================================
        #
        # These parameters can be changed while the node
        # is running.
        #
        # Examples:
        #
        # ros2 param set /ultrasonic_avoidance_node \
        #     escape_speed 15
        #
        # ros2 param set /ultrasonic_avoidance_node \
        #     escape_duration 0.8
        #
        # ros2 param set /ultrasonic_avoidance_node \
        #     block_distance 0.30
        #

        self.declare_parameter('block_distance', 0.20)
        self.declare_parameter('required_detections', 3)
        self.declare_parameter('escape_speed', 10)
        self.declare_parameter('escape_duration', 0.5)

        # =====================================================
        # HEXAGON SENSOR GEOMETRY
        # =====================================================
        #
        #                     FRONT
        #                       ^
        #
        #                      S2
        #                 __________
        #                /          \
        #            S1 /            \ S3
        #              /              \
        #              \              /
        #            S6 \            / S4
        #                \__________/
        #                     S5
        #
        #
        # Robot coordinate system:
        #
        # +X = forward
        # +Y = left
        #
        #
        # Sensor directions:
        #
        # S2 =    0 degrees
        # S1 =   60 degrees
        # S6 =  120 degrees
        # S5 =  180 degrees
        # S4 = -120 degrees
        # S3 =  -60 degrees
        #

        self.sensor_angles = {
            1: math.radians(60.0),
            2: math.radians(0.0),
            3: math.radians(-60.0),
            4: math.radians(-120.0),
            5: math.radians(180.0),
            6: math.radians(120.0),
        }

        # =====================================================
        # REAL SENSOR DATA
        # =====================================================

        self.sensor_ranges = {
            1: None,
            2: None,
            3: None,
            4: None,
            5: None,
            6: None,
        }

        # =====================================================
        # MANUAL SENSOR STATES
        # =====================================================
        #
        # S1, S2, S3 = real HC-SR04 sensors
        #
        # S4, S5, S6 = currently manual
        #
        # False = CLEAR
        # True  = BLOCKED
        #

        self.manual_states = {
            4: False,
            5: False,
            6: False,
        }

        # When a manual sensor is marked BLOCKED,
        # pretend the obstacle is this distance away.
        #
        # This allows the manual sensors to participate
        # in the proportional vector calculation.

        self.manual_block_distance = 0.10

        # =====================================================
        # DETECTION STATE
        # =====================================================

        self.consecutive_blocked = 0

        # =====================================================
        # ESCAPE STATE
        # =====================================================

        self.escaping = False
        self.escape_start_time = None

        # After escaping, the blocked group must disappear
        # before another escape can trigger.

        self.waiting_for_clear = False

        # =====================================================
        # REAL HC-SR04 SUBSCRIBERS
        # =====================================================

        self.create_subscription(
            Range,
            '/ultrasonic/sensor_1',
            self.sensor1_callback,
            10
        )

        self.create_subscription(
            Range,
            '/ultrasonic/sensor_2',
            self.sensor2_callback,
            10
        )

        self.create_subscription(
            Range,
            '/ultrasonic/sensor_3',
            self.sensor3_callback,
            10
        )

        # =====================================================
        # MANUAL SENSOR SUBSCRIBERS
        # =====================================================
        #
        # 0 = CLEAR
        # 1 = BLOCKED
        #

        self.create_subscription(
            Int32,
            '/ultrasonic/manual_sensor_4',
            self.manual_sensor4_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/ultrasonic/manual_sensor_5',
            self.manual_sensor5_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/ultrasonic/manual_sensor_6',
            self.manual_sensor6_callback,
            10
        )

        # =====================================================
        # MOTOR PUBLISHER
        # =====================================================

        self.motor_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_speeds',
            10
        )

        # =====================================================
        # ESCAPE TIMER
        # =====================================================

        self.escape_timer = self.create_timer(
            0.05,
            self.escape_timer_callback
        )

        # =====================================================
        # STARTUP INFORMATION
        # =====================================================

        self.get_logger().info(
            'Six-position ultrasonic avoidance controller started.'
        )

        self.get_logger().info(
            'Physical sensors: S1 S2 S3'
        )

        self.get_logger().info(
            'Manual sensors: S4 S5 S6'
        )

        self.print_current_parameters()

    # =========================================================
    # PARAMETER HELPERS
    # =========================================================

    def get_block_distance(self):

        return float(
            self.get_parameter(
                'block_distance'
            ).value
        )

    def get_required_detections(self):

        return int(
            self.get_parameter(
                'required_detections'
            ).value
        )

    def get_escape_speed(self):

        speed = int(
            self.get_parameter(
                'escape_speed'
            ).value
        )

        # Keep speed inside the Hiwonder limit.

        return max(
            0,
            min(50, abs(speed))
        )

    def get_escape_duration(self):

        duration = float(
            self.get_parameter(
                'escape_duration'
            ).value
        )

        return max(
            0.0,
            duration
        )

    def print_current_parameters(self):

        self.get_logger().info(
            f'Block distance: '
            f'{self.get_block_distance():.2f} m'
        )

        self.get_logger().info(
            f'Required detections: '
            f'{self.get_required_detections()}'
        )

        self.get_logger().info(
            f'Escape speed: '
            f'{self.get_escape_speed()}'
        )

        self.get_logger().info(
            f'Escape duration: '
            f'{self.get_escape_duration():.2f} s'
        )

    # =========================================================
    # REAL SENSOR CALLBACKS
    # =========================================================

    def sensor1_callback(self, msg):

        self.sensor_ranges[1] = float(
            msg.range
        )

    def sensor2_callback(self, msg):

        self.sensor_ranges[2] = float(
            msg.range
        )

    def sensor3_callback(self, msg):

        self.sensor_ranges[3] = float(
            msg.range
        )

        # HC-SR04 driver publishes sensors sequentially:
        #
        # S1 -> S2 -> S3
        #
        # Therefore receiving S3 means we have completed
        # one new physical sensor scan.

        self.evaluate_complete_scan()

    # =========================================================
    # MANUAL SENSOR CALLBACKS
    # =========================================================

    def manual_sensor4_callback(self, msg):

        self.manual_states[4] = (
            msg.data != 0
        )

        self.get_logger().warning(
            f'Manual S4 = '
            f'{"BLOCKED" if self.manual_states[4] else "CLEAR"}'
        )

    def manual_sensor5_callback(self, msg):

        self.manual_states[5] = (
            msg.data != 0
        )

        self.get_logger().warning(
            f'Manual S5 = '
            f'{"BLOCKED" if self.manual_states[5] else "CLEAR"}'
        )

    def manual_sensor6_callback(self, msg):

        self.manual_states[6] = (
            msg.data != 0
        )

        self.get_logger().warning(
            f'Manual S6 = '
            f'{"BLOCKED" if self.manual_states[6] else "CLEAR"}'
        )

    # =========================================================
    # RANGE VALIDATION
    # =========================================================

    def valid_range(self, distance):

        if distance is None:
            return False

        if not math.isfinite(distance):
            return False

        if distance < 0.02:
            return False

        if distance > 4.0:
            return False

        return True

    # =========================================================
    # SENSOR BLOCKED STATE
    # =========================================================

    def sensor_blocked(
        self,
        sensor_id
    ):

        # -----------------------------------------------------
        # Physical sensors
        # -----------------------------------------------------

        if sensor_id in (
            1,
            2,
            3
        ):

            distance = (
                self.sensor_ranges[
                    sensor_id
                ]
            )

            if not self.valid_range(
                distance
            ):
                return False

            # Read CURRENT parameter value.
            #
            # Therefore:
            #
            # ros2 param set ... block_distance 0.30
            #
            # immediately changes the threshold.

            block_distance = (
                self.get_block_distance()
            )

            return (
                distance
                <= block_distance
            )

        # -----------------------------------------------------
        # Manual sensors
        # -----------------------------------------------------

        return self.manual_states[
            sensor_id
        ]

    # =========================================================
    # SENSOR DISTANCE FOR VECTOR CALCULATION
    # =========================================================

    def get_sensor_distance(
        self,
        sensor_id
    ):

        # -----------------------------------------------------
        # Real sensor
        # -----------------------------------------------------

        if sensor_id in (
            1,
            2,
            3
        ):

            distance = (
                self.sensor_ranges[
                    sensor_id
                ]
            )

            if self.valid_range(
                distance
            ):
                return distance

            return None

        # -----------------------------------------------------
        # Manual sensor
        # -----------------------------------------------------

        if self.manual_states[
            sensor_id
        ]:

            return (
                self.manual_block_distance
            )

        return None

    # =========================================================
    # FIND CONSECUTIVE BLOCKED GROUPS
    # =========================================================

    def find_blocked_groups(self):

        # Six possible groups of three consecutive
        # hexagon edges.

        groups = [
            (1, 2, 3),
            (2, 3, 4),
            (3, 4, 5),
            (4, 5, 6),
            (5, 6, 1),
            (6, 1, 2),
        ]

        blocked_groups = []

        for group in groups:

            if all(
                self.sensor_blocked(
                    sensor
                )
                for sensor in group
            ):

                blocked_groups.append(
                    group
                )

        return blocked_groups

    # =========================================================
    # COMPLETE SCAN EVALUATION
    # =========================================================

    def evaluate_complete_scan(self):

        # -----------------------------------------------------
        # Verify the physical sensors
        # -----------------------------------------------------

        for sensor_id in (
            1,
            2,
            3
        ):

            if not self.valid_range(
                self.sensor_ranges[
                    sensor_id
                ]
            ):

                self.consecutive_blocked = 0

                self.get_logger().warning(
                    f'Invalid reading '
                    f'from S{sensor_id}.'
                )

                return

        # -----------------------------------------------------
        # Determine all six states
        # -----------------------------------------------------

        states = {}

        for sensor_id in range(
            1,
            7
        ):

            states[sensor_id] = (
                self.sensor_blocked(
                    sensor_id
                )
            )

        # -----------------------------------------------------
        # Print sensor states
        # -----------------------------------------------------

        state_string = ' '.join(
            f'S{i}='
            f'{"B" if states[i] else "C"}'
            for i in range(
                1,
                7
            )
        )

        self.get_logger().info(
            state_string
        )

        # -----------------------------------------------------
        # Find three-edge blocked groups
        # -----------------------------------------------------

        blocked_groups = (
            self.find_blocked_groups()
        )

        has_blocked_group = (
            len(blocked_groups) > 0
        )

        # -----------------------------------------------------
        # Already escaping
        # -----------------------------------------------------

        if self.escaping:
            return

        # -----------------------------------------------------
        # Waiting for obstacle pattern to clear
        # -----------------------------------------------------

        if self.waiting_for_clear:

            if not has_blocked_group:

                self.waiting_for_clear = False

                self.consecutive_blocked = 0

                self.get_logger().info(
                    'Blocked group cleared. '
                    'Controller re-armed.'
                )

            return

        # -----------------------------------------------------
        # Consecutive scan counter
        # -----------------------------------------------------

        if has_blocked_group:

            self.consecutive_blocked += 1

            required = (
                self.get_required_detections()
            )

            groups_string = ', '.join(
                str(group)
                for group
                in blocked_groups
            )

            self.get_logger().warning(
                f'Blocked group detection '
                f'{self.consecutive_blocked}/'
                f'{required} '
                f'Groups: '
                f'{groups_string}'
            )

        else:

            if (
                self.consecutive_blocked
                > 0
            ):

                self.get_logger().info(
                    'Blocked sequence '
                    'interrupted.'
                )

            self.consecutive_blocked = 0

        # -----------------------------------------------------
        # Trigger escape
        # -----------------------------------------------------

        required = (
            self.get_required_detections()
        )

        if (
            self.consecutive_blocked
            >= required
        ):

            self.consecutive_blocked = 0

            self.start_escape()

    # =========================================================
    # CALCULATE ESCAPE VECTOR
    # =========================================================

    def calculate_escape_vector(self):

        escape_x = 0.0
        escape_y = 0.0

        block_distance = (
            self.get_block_distance()
        )

        # Safety against division by zero.

        if block_distance <= 0.0:

            self.get_logger().error(
                'block_distance must '
                'be greater than zero.'
            )

            return None

        # -----------------------------------------------------
        # Add repulsive vector from every blocked sensor
        # -----------------------------------------------------

        for sensor_id in range(
            1,
            7
        ):

            if not self.sensor_blocked(
                sensor_id
            ):
                continue

            distance = (
                self.get_sensor_distance(
                    sensor_id
                )
            )

            if distance is None:
                continue

            # -------------------------------------------------
            # Proportional obstacle strength
            # -------------------------------------------------
            #
            # Closer obstacle:
            #     stronger repulsion
            #
            # Obstacle near threshold:
            #     weaker repulsion
            #

            strength = (
                block_distance
                - distance
            ) / block_distance

            # Every blocked sensor gets at least
            # a small contribution.

            strength = max(
                0.10,
                min(
                    1.0,
                    strength
                )
            )

            angle = (
                self.sensor_angles[
                    sensor_id
                ]
            )

            # Sensor points toward obstacle.
            #
            # Negative vector points AWAY
            # from obstacle.

            escape_x += (
                -strength
                * math.cos(angle)
            )

            escape_y += (
                -strength
                * math.sin(angle)
            )

        # -----------------------------------------------------
        # Normalize final vector
        # -----------------------------------------------------

        magnitude = math.hypot(
            escape_x,
            escape_y
        )

        if magnitude < 0.001:

            self.get_logger().error(
                'Escape vectors cancelled '
                'each other.'
            )

            return None

        escape_x /= magnitude
        escape_y /= magnitude

        # -----------------------------------------------------
        # Calculate human-readable angle
        # -----------------------------------------------------

        angle_deg = math.degrees(
            math.atan2(
                escape_y,
                escape_x
            )
        )

        self.get_logger().warning(
            f'Escape vector: '
            f'X={escape_x:.3f}, '
            f'Y={escape_y:.3f}, '
            f'angle={angle_deg:.1f} deg'
        )

        return (
            escape_x,
            escape_y
        )

    # =========================================================
    # START ESCAPE
    # =========================================================

    def start_escape(self):

        if self.escaping:
            return

        vector = (
            self.calculate_escape_vector()
        )

        if vector is None:

            self.get_logger().error(
                'No valid escape direction.'
            )

            self.stop_vehicle()

            return

        vx, vy = vector

        # Read the speed NOW.
        #
        # This means the latest:
        #
        # ros2 param set ... escape_speed X
        #
        # will be used.

        current_speed = (
            self.get_escape_speed()
        )

        current_duration = (
            self.get_escape_duration()
        )

        self.get_logger().warning(
            'THREE CONSECUTIVE '
            'EDGES BLOCKED!'
        )

        self.get_logger().warning(
            f'Starting escape: '
            f'speed={current_speed}, '
            f'duration='
            f'{current_duration:.2f}s'
        )

        self.escaping = True

        self.escape_start_time = (
            self.get_clock().now()
        )

        self.move_mecanum(
            vx,
            vy
        )

    # =========================================================
    # MECANUM MOVEMENT
    # =========================================================

    def move_mecanum(
        self,
        vx,
        vy
    ):

        # -----------------------------------------------------
        # IMPORTANT:
        #
        # Read current escape_speed parameter every time
        # an escape starts.
        #
        # Therefore the speed can be changed while ROS
        # is running.
        # -----------------------------------------------------

        speed = float(
            self.get_escape_speed()
        )

        if speed <= 0.0:

            self.get_logger().warning(
                'Escape speed is zero. '
                'Vehicle will not move.'
            )

            self.stop_vehicle()

            return

        self.get_logger().info(
            f'Using escape speed: '
            f'{speed:.0f}'
        )

        # -----------------------------------------------------
        # Scale normalized escape vector
        # -----------------------------------------------------

        vx *= speed
        vy *= speed

        # -----------------------------------------------------
        # Mecanum inverse kinematics
        # -----------------------------------------------------
        #
        # Robot coordinates:
        #
        # +X = forward
        # +Y = left
        #
        # No rotation is commanded.
        #

        fl = vx - vy
        fr = vx + vy
        rl = vx + vy
        rr = vx - vy

        # -----------------------------------------------------
        # Normalize wheel commands
        # -----------------------------------------------------

        maximum = max(
            abs(fl),
            abs(fr),
            abs(rl),
            abs(rr),
            1.0
        )

        if maximum > speed:

            scale = (
                speed
                / maximum
            )

            fl *= scale
            fr *= scale
            rl *= scale
            rr *= scale

        # -----------------------------------------------------
        # VERIFIED MOTOR CONFIGURATION
        # -----------------------------------------------------
        #
        # Motor locations:
        #
        #                 FRONT
        #
        #          M4             M2
        #      Front Left     Front Right
        #
        #
        #          M1             M3
        #       Rear Left      Rear Right
        #
        #
        # Verified polarity:
        #
        # M1 + = forward
        # M2 + = forward
        # M3 - = forward
        # M4 - = forward
        #
        #
        # Hiwonder order:
        #
        # [M1, M2, M3, M4]
        #

        m1 = rl
        m2 = fr

        # Reverse polarity for M3/M4.

        m3 = -rr
        m4 = -fl

        self.publish_motor_speeds(
            m1,
            m2,
            m3,
            m4
        )

    # =========================================================
    # MOTOR PUBLISHER
    # =========================================================

    def publish_motor_speeds(
        self,
        m1,
        m2,
        m3,
        m4
    ):

        # Hiwonder motor node accepts:
        #
        # -50 ... +50

        m1 = max(
            -50,
            min(
                50,
                int(round(m1))
            )
        )

        m2 = max(
            -50,
            min(
                50,
                int(round(m2))
            )
        )

        m3 = max(
            -50,
            min(
                50,
                int(round(m3))
            )
        )

        m4 = max(
            -50,
            min(
                50,
                int(round(m4))
            )
        )

        msg = Int32MultiArray()

        msg.data = [
            m1,
            m2,
            m3,
            m4
        ]

        self.motor_pub.publish(
            msg
        )

        self.get_logger().warning(
            f'Motor command: '
            f'[{m1}, {m2}, '
            f'{m3}, {m4}]'
        )

    # =========================================================
    # ESCAPE TIMER
    # =========================================================

    def escape_timer_callback(self):

        if not self.escaping:
            return

        if (
            self.escape_start_time
            is None
        ):
            return

        now = (
            self.get_clock().now()
        )

        elapsed = (
            now
            - self.escape_start_time
        ).nanoseconds / 1e9

        # -----------------------------------------------------
        # Read CURRENT duration parameter.
        #
        # Therefore escape_duration can also be changed
        # while the node is running.
        # -----------------------------------------------------

        escape_duration = (
            self.get_escape_duration()
        )

        if (
            elapsed
            >= escape_duration
        ):

            self.stop_vehicle()

            self.escaping = False

            self.escape_start_time = None

            self.waiting_for_clear = True

            self.get_logger().info(
                'Escape completed.'
            )

            self.get_logger().info(
                'Waiting for blocked '
                'group to clear.'
            )

    # =========================================================
    # STOP VEHICLE
    # =========================================================

    def stop_vehicle(self):

        self.publish_motor_speeds(
            0,
            0,
            0,
            0
        )

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def destroy_node(self):

        self.get_logger().info(
            'Stopping vehicle.'
        )

        self.stop_vehicle()

        super().destroy_node()


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        UltrasonicAvoidanceNode()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        try:

            node.stop_vehicle()

        except Exception:

            pass

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()