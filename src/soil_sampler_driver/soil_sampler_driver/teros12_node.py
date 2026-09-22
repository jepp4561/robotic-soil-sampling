import re

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

import serial


class Teros12Node(Node):

    def __init__(self):
        super().__init__("teros12")

        self.declare_parameter("port", "/dev/serial/by-id/usb-Apogee_Instruments__Inc._Sensor_Interface_B16392460F001A00-if00")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("address", "0")
        self.declare_parameter("frame_id", "soil_sensor")
        self.declare_parameter("measurement_period", 10.0)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.address = self.get_parameter("address").value
        self.frame_id = self.get_parameter("frame_id").value
        self.measurement_period = self.get_parameter("measurement_period").value

        self.vwc_publisher = self.create_publisher(Float32, "teros12/volumetric_water_content", 10)
        self.temperature_publisher = self.create_publisher(Temperature, "teros12/temperature", 10)
        self.ec_publisher = self.create_publisher(Float32, "teros12/electrical_conductivity", 10)

        self.serial = None

        self.connect()

        self.timer = self.create_timer(self.measurement_period, self.measurement_callback)

    def connect(self):
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=2.0)

            self.get_logger().info(f"Connected to TEROS 12 on {self.port}")

            response = self.send_command(f"{self.address}I!")

            if response is not None:
                self.get_logger().info(f"TEROS 12 identification: {response!r}")

        except serial.SerialException as error:
            self.get_logger().error(f"Failed to open TEROS 12 serial port: {error}")
            self.serial = None

    def send_command(self, command):
        if self.serial is None or not self.serial.is_open:
            return None

        try:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self.serial.write((command + "\r\n").encode())
            self.serial.flush()

            response = self.serial.readline().decode(errors="replace")

            return response

        except serial.SerialException as error:
            self.get_logger().error(f"Serial communication error: {error}")
            return None

    def measurement_callback(self):
        if self.serial is None or not self.serial.is_open:
            self.connect()

            if self.serial is None:
                return

        response = self.send_command(f"{self.address}XR3!")

        if response is None:
            return

        self.get_logger().debug(f"Raw TEROS 12 response: {response!r}")

        self.parse_measurement(response)

    def parse_measurement(self, response):
        response = response.split("\r")[0].split("\n")[0]

        number_pattern = r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?"

        values = re.findall(number_pattern, response)

        if len(values) < 3:
            self.get_logger().warning(f"Could not find three measurements in response: {response!r}")
            return

        try:
            raw_vwc = float(values[-3])
            vwc = 3.879e-4 * raw_vwc - 0.6956 # METER's calibration equation for vwc
            temperature = float(values[-2])
            ec = float(values[-1])
        except ValueError:
            self.get_logger().warning(f"Could not parse measurement values: {response!r}")
            return

        now = self.get_clock().now().to_msg()

        vwc_message = Float32()
        vwc_message.data = vwc

        temperature_message = Temperature()
        temperature_message.header.stamp = now
        temperature_message.header.frame_id = self.frame_id
        temperature_message.temperature = temperature

        ec_message = Float32()
        ec_message.data = ec

        self.vwc_publisher.publish(vwc_message)
        self.temperature_publisher.publish(temperature_message)
        self.ec_publisher.publish(ec_message)

        # self.get_logger().info(
        #     f"TEROS 12: VWC={vwc:.2f}, "
        #     f"temperature={temperature:.2f}, "
        #     f"EC={ec:.3f}"
        # )

    def destroy_node(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Teros12Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
