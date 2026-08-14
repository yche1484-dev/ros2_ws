import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import lgpio

class HCSR04DriverNode(Node):
    def __init__(self):
        super().__init__('hcsr04_driver_node')

        # Publishers
        self.pub_sensor1 = self.create_publisher(Range, '/ultrasonic/sensor_1', 10)
        self.pub_sensor2 = self.create_publisher(Range, '/ultrasonic/sensor_2', 10)
        self.pub_sensor3 = self.create_publisher(Range, '/ultrasonic/sensor_3', 10)

        # Pin mapping (BCM)
        self.sensors = [
            {'name': 'Sensor 1', 'trig': 25, 'echo': 20, 'pub': self.pub_sensor1, 'frame': 'sensor_1_link', 'last_valid': 0.0},
            {'name': 'Sensor 2', 'trig': 5,  'echo': 6,  'pub': self.pub_sensor2, 'frame': 'sensor_2_link', 'last_valid': 0.0},
            {'name': 'Sensor 3', 'trig': 16, 'echo': 26, 'pub': self.pub_sensor3, 'frame': 'sensor_3_link', 'last_valid': 0.0},
        ]

        # Auto-detect GPIO Chip
        self.chip_handle = None
        for chip in [4, 0]:
            try:
                self.chip_handle = lgpio.gpiochip_open(chip)
                self.get_logger().info(f'Initialized GPIO on gpiochip{chip}')
                break
            except lgpio.error:
                continue

        if self.chip_handle is None:
            self.get_logger().error('Could not open GPIO chip.')
            raise RuntimeError('GPIO initialization failed.')

        # Claim GPIO pins
        for s in self.sensors:
            self.init_sensor_pins(s['trig'], s['echo'])

        # Timer set to 5 Hz (0.2s)
        self.timer = self.create_timer(0.2, self.publish_sensor_data)
        self.get_logger().info('HC-SR04 Robust Driver Node active with Auto-Recovery.')

    def init_sensor_pins(self, trig_pin: int, echo_pin: int):
        """Claims and sets standard pin modes."""
        try:
            lgpio.gpio_free(self.chip_handle, trig_pin)
            lgpio.gpio_free(self.chip_handle, echo_pin)
        except Exception:
            pass

        lgpio.gpio_claim_output(self.chip_handle, trig_pin)
        lgpio.gpio_write(self.chip_handle, trig_pin, 0)
        lgpio.gpio_claim_input(self.chip_handle, echo_pin)

    def recover_hung_pin(self, echo_pin: int):
        """Forces a hung ECHO pin LOW to clear sensor latch-up."""
        try:
            lgpio.gpio_free(self.chip_handle, echo_pin)
            lgpio.gpio_claim_output(self.chip_handle, echo_pin)
            lgpio.gpio_write(self.chip_handle, echo_pin, 0)
            time.sleep(0.002)
            lgpio.gpio_free(self.chip_handle, echo_pin)
            lgpio.gpio_claim_input(self.chip_handle, echo_pin)
        except Exception:
            pass

    def single_measure(self, trig_pin: int, echo_pin: int) -> float:
        # Pre-fire reset check
        lgpio.gpio_write(self.chip_handle, trig_pin, 0)
        time.sleep(0.001)

        if lgpio.gpio_read(self.chip_handle, echo_pin) == 1:
            return float('nan')

        # Trigger pulse
        lgpio.gpio_write(self.chip_handle, trig_pin, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip_handle, trig_pin, 0)

        # Pulse rise wait (25ms timeout)
        start_time = time.time()
        pulse_start = start_time
        while lgpio.gpio_read(self.chip_handle, echo_pin) == 0:
            pulse_start = time.time()
            if (pulse_start - start_time) > 0.025:
                return float('nan')

        # Pulse fall wait (25ms timeout)
        pulse_end = pulse_start
        while lgpio.gpio_read(self.chip_handle, echo_pin) == 1:
            pulse_end = time.time()
            if (pulse_end - pulse_start) > 0.025:
                return float('nan')

        duration = pulse_end - pulse_start
        return (duration * 343.0) / 2.0

    def measure_distance_robust(self, trig_pin: int, echo_pin: int) -> float:
        """Attempts reading up to 3 times before declaring a failure."""
        for attempt in range(3):
            dist = self.single_measure(trig_pin, echo_pin)
            if dist == dist and 0.02 <= dist <= 4.0:  # Valid reading check
                return dist
            
            # If measurement failed, force-unlatch the pin and pause before retrying
            self.recover_hung_pin(echo_pin)
            time.sleep(0.005)

        return float('nan')

    def publish_sensor_data(self):
        for s in self.sensors:
            dist = self.measure_distance_robust(s['trig'], s['echo'])

            if dist != dist:  # Check for NaN
                # Fall back to previous valid reading if available, else 0.0
                final_range = s['last_valid']
            else:
                final_range = float(dist)
                s['last_valid'] = final_range

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = s['frame']
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = final_range

            s['pub'].publish(msg)

            # Delay to allow echoes from one sensor to fade before triggering the next
            time.sleep(0.015)

    def destroy_node(self):
        if hasattr(self, 'chip_handle') and self.chip_handle is not None:
            for s in self.sensors:
                try:
                    lgpio.gpio_free(self.chip_handle, s['trig'])
                    lgpio.gpio_free(self.chip_handle, s['echo'])
                except Exception:
                    pass
            lgpio.gpiochip_close(self.chip_handle)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HCSR04DriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()