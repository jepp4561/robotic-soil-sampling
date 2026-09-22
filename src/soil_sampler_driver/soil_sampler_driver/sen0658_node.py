import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, Illuminance
from std_msgs.msg import Float32, Int32

import serial


def modbus_crc16(data: bytes) -> bytes:
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1

    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def to_signed16(value):
    if value >= 0x8000:
        value -= 0x10000

    return value


class Sen0658Node(Node):
    # (start_register, register_count) for each group of related readings.
    # Register 0x01F5 is reserved/unused and simply skipped over when parsing the "wind" block below.
    WIND_BLOCK = (0x01F4, 4)  # wind speed, (reserved), wind sector, wind degrees
    HUMITURE_NOISE_BLOCK = (0x01F8, 3)  # humidity, temperature, noise
    LIGHT_BLOCK = (0x01FE, 2)  # illuminance high word, illuminance low word
    PM_PRESSURE_BLOCK = (0x01FB, 3)  # PM2.5, PM10, atmospheric pressure
    RAINFALL_BLOCK = (0x0201, 1)  # rainfall

    def __init__(self):
        super().__init__("sen0658")

        self.declare_parameter("port", "/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FT54EVY9-if00-port0")
        self.declare_parameter("baudrate", 4800)
        self.declare_parameter("address", 1)
        self.declare_parameter("frame_id", "weather_station")
        self.declare_parameter("measurement_period", 10.0)
        self.declare_parameter("read_timeout", 1.0)
        self.declare_parameter("wind_speed_scale", 100.0) # may be 10.0, who knows

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.address = self.get_parameter("address").value
        self.frame_id = self.get_parameter("frame_id").value
        self.measurement_period = self.get_parameter("measurement_period").value
        self.read_timeout = self.get_parameter("read_timeout").value
        self.wind_speed_scale = self.get_parameter("wind_speed_scale").value

        self.wind_speed_publisher = self.create_publisher(Float32, "sen0658/wind_speed", 10)
        self.wind_direction_sector_publisher = self.create_publisher(Int32, "sen0658/wind_direction_gear", 10)
        self.wind_direction_publisher = self.create_publisher(Float32, "sen0658/wind_direction", 10)
        self.humidity_publisher = self.create_publisher(RelativeHumidity, "sen0658/humidity", 10)
        self.temperature_publisher = self.create_publisher(Temperature, "sen0658/temperature", 10)
        self.noise_publisher = self.create_publisher(Float32, "sen0658/noise", 10)
        self.illuminance_publisher = self.create_publisher(Illuminance, "sen0658/illuminance", 10)
        self.pm2_5_publisher = self.create_publisher(Float32, "sen0658/pm2_5", 10)
        self.pm10_publisher = self.create_publisher(Float32, "sen0658/pm10", 10)
        self.pressure_publisher = self.create_publisher(FluidPressure, "sen0658/atmospheric_pressure", 10)
        self.rainfall_publisher = self.create_publisher(Float32, "sen0658/rainfall", 10)

        self.serial = None

        self.connect()

        self.timer = self.create_timer(self.measurement_period, self.measurement_callback)

    def connect(self):
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=self.read_timeout)

            self.get_logger().info(f"Connected to SEN0658 on {self.port}")

            # Sanity-check the link by reading the device address register.
            registers = self.read_registers(0x07D0, 1)

            if registers is not None:
                self.get_logger().info(f"SEN0658 reports device address: {registers[0]}")

        except serial.SerialException as error:
            self.get_logger().error(f"Failed to open SEN0658 serial port: {error}")
            self.serial = None

    def read_registers(self, start_register, count):
        """Issue a Modbus-RTU "read holding registers" (function 0x03)
        request and return the parsed list of unsigned 16-bit register
        values, or None on any communication/framing error."""
        if self.serial is None or not self.serial.is_open:
            return None

        request = bytes([self.address, 0x03]) + start_register.to_bytes(2, "big") + count.to_bytes(2, "big")
        request += modbus_crc16(request)

        expected_length = 5 + count * 2  # address + function + byte count + data + crc

        try:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self.serial.write(request)
            self.serial.flush()

            response = self.serial.read(expected_length)

        except serial.SerialException as error:
            self.get_logger().error(f"Serial communication error: {error}")
            return None

        if len(response) != expected_length:
            self.get_logger().warning(f"Short/no response reading registers 0x{start_register:04X} (got {len(response)} of {expected_length} bytes)")
            return None

        if response[0] != self.address:
            self.get_logger().warning(f"Unexpected address in response: {response[0]!r}")
            return None

        if response[1] == (0x03 | 0x80):
            self.get_logger().warning(f"SEN0658 returned Modbus exception code {response[2]} for registers 0x{start_register:04X}")
            return None

        if response[1] != 0x03:
            self.get_logger().warning(f"Unexpected function code in response: {response[1]!r}")
            return None

        byte_count = response[2]

        if byte_count != count * 2:
            self.get_logger().warning(f"Unexpected byte count in response: {byte_count}")
            return None

        if response[-2:] != modbus_crc16(response[:-2]):
            self.get_logger().warning(f"CRC mismatch reading registers 0x{start_register:04X}")
            return None

        data = response[3:3 + byte_count]

        return [int.from_bytes(data[i:i + 2], "big") for i in range(0, byte_count, 2)]

    def measurement_callback(self):
        if self.serial is None or not self.serial.is_open:
            self.connect()

            if self.serial is None:
                return

        self.read_and_publish_wind()
        self.read_and_publish_humiture_noise()
        self.read_and_publish_light()
        self.read_and_publish_pm_pressure()
        self.read_and_publish_rainfall()

    def read_and_publish_wind(self):
        start_register, count = self.WIND_BLOCK
        registers = self.read_registers(start_register, count)

        if registers is None:
            return

        wind_speed = registers[0] / self.wind_speed_scale
        # registers[1] (0x01F5) is reserved and skipped.
        wind_direction_sector = registers[2]
        wind_direction = registers[3]

        wind_speed_message = Float32()
        wind_speed_message.data = wind_speed

        wind_direction_sector_message = Int32()
        wind_direction_sector_message.data = wind_direction_sector

        wind_direction_message = Float32()
        wind_direction_message.data = float(wind_direction)

        self.wind_speed_publisher.publish(wind_speed_message)
        self.wind_direction_sector_publisher.publish(wind_direction_sector_message)
        self.wind_direction_publisher.publish(wind_direction_message)

    def read_and_publish_humiture_noise(self):
        start_register, count = self.HUMITURE_NOISE_BLOCK
        registers = self.read_registers(start_register, count)

        if registers is None:
            return

        humidity = registers[0] / 10.0  # %RH
        temperature = to_signed16(registers[1]) / 10.0  # degrees C
        noise = registers[2] / 10.0  # dB

        now = self.get_clock().now().to_msg()

        humidity_message = RelativeHumidity()
        humidity_message.header.stamp = now
        humidity_message.header.frame_id = self.frame_id
        humidity_message.relative_humidity = humidity / 100.0  # 0.0-1.0

        temperature_message = Temperature()
        temperature_message.header.stamp = now
        temperature_message.header.frame_id = self.frame_id
        temperature_message.temperature = temperature

        noise_message = Float32()
        noise_message.data = noise

        self.humidity_publisher.publish(humidity_message)
        self.temperature_publisher.publish(temperature_message)
        self.noise_publisher.publish(noise_message)

    def read_and_publish_light(self):
        start_register, count = self.LIGHT_BLOCK
        registers = self.read_registers(start_register, count)

        if registers is None:
            return

        illuminance = (registers[0] << 16) | registers[1]  # lux

        now = self.get_clock().now().to_msg()

        illuminance_message = Illuminance()
        illuminance_message.header.stamp = now
        illuminance_message.header.frame_id = self.frame_id
        illuminance_message.illuminance = float(illuminance)

        self.illuminance_publisher.publish(illuminance_message)

    def read_and_publish_pm_pressure(self):
        start_register, count = self.PM_PRESSURE_BLOCK
        registers = self.read_registers(start_register, count)

        if registers is None:
            return

        pm2_5 = registers[0]  # ug/m3
        pm10 = registers[1]  # ug/m3
        # DFRobot's example code labels this "kPa" after dividing by 10,
        # but a raw value on the order of 10130 dividing down to ~1013.0
        # reads far more like hPa; treat it as hPa here and convert to
        # the Pascals expected by sensor_msgs/FluidPressure.
        pressure_hpa = registers[2] / 10.0

        pm2_5_message = Float32()
        pm2_5_message.data = float(pm2_5)

        pm10_message = Float32()
        pm10_message.data = float(pm10)

        now = self.get_clock().now().to_msg()

        pressure_message = FluidPressure()
        pressure_message.header.stamp = now
        pressure_message.header.frame_id = self.frame_id
        pressure_message.fluid_pressure = pressure_hpa * 100.0  # hPa -> Pa

        self.pm2_5_publisher.publish(pm2_5_message)
        self.pm10_publisher.publish(pm10_message)
        self.pressure_publisher.publish(pressure_message)

    def read_and_publish_rainfall(self):
        start_register, count = self.RAINFALL_BLOCK
        registers = self.read_registers(start_register, count)

        if registers is None:
            return

        rainfall = registers[0] / 10.0  # mm

        rainfall_message = Float32()
        rainfall_message.data = rainfall

        self.rainfall_publisher.publish(rainfall_message)

    def destroy_node(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Sen0658Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()