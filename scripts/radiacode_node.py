#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from radiacode import RadiaCode, RealTimeData
from radiacode.transports.usb import DeviceNotFound as DeviceNotFoundUSB
from radiacode.transports.bluetooth import DeviceNotFound as DeviceNotFoundBT
from std_msgs.msg import String, Float32

from radiacode_driver.msg import Spectrum
from builtin_interfaces.msg import Duration

class RadiacodeNode(Node):
    def __init__(self):
        super().__init__('radiacode_node')
        
        # Declare ROS2 parameters with default values
        self.declare_parameter('bluetooth_mac', '')
        self.declare_parameter('serial', '')
        self.declare_parameter('frequency', 2)  # Default frequency of 2 Hz (0.5 seconds)
        
        # Retrieve parameters
        bluetooth_mac = self.get_parameter('bluetooth_mac').value
        serial = self.get_parameter('serial').value
        frequency = self.get_parameter('frequency').value
        
        # Validate frequency parameter
        if frequency <= 0:
            self.get_logger().error('Frequency must be greater than 0. Using default value of 0.5 Hz.')
            frequency = 0.5
        
        # Convert frequency to period (in seconds) for the timer
        period = 1.0 / frequency
        
        # Setup publisher
        self.spectrum_publisher_ = self.create_publisher(Spectrum, 'spectrum', 10)
        self.dose_rate_publisher_ = self.create_publisher(Float32, 'dose_rate', 10)
        self.count_rate_publisher_ = self.create_publisher(Float32, 'count_rate', 10)
        
        # Initialize RadiaCode device
        if bluetooth_mac:
            self.get_logger().info(f'Connecting to Radiacode via Bluetooth (MAC address: {bluetooth_mac})')
            try:
                self.rc = RadiaCode(bluetooth_mac=bluetooth_mac)
            except DeviceNotFoundBT as e:
                self.get_logger().error(str(e))
                return
            except ValueError as e:
                self.get_logger().error(str(e))
                return
        else:
            self.get_logger().info('Connecting to Radiacode via USB' + (f' (serial number: {serial})' if serial else ''))
            try:
                self.rc = RadiaCode(serial_number=serial)
            except DeviceNotFoundUSB:
                self.get_logger().error('Device not found, check your USB connection')
                return
        
        # Log device info
        self.get_logger().info(f'### Serial number: {self.rc.serial_number()}')
        self.get_logger().info(f'### Firmware: {self.rc.fw_version()}')
        
        # Start publishing spectrum data
        self.timer = self.create_timer(period, self.publish_spectrum_data)
        self.get_logger().info(f'Publishing radiation data at {frequency} Hz')
    
    def publish_spectrum_data(self):
        spectrum_msg = Spectrum()
        dose_rate_msg = Float32()
        count_rate_msg = Float32()

        spectrum = self.rc.spectrum()
        spectrum_msg.duration = Duration()
        spectrum_msg.duration.sec = int(spectrum.duration.total_seconds())
        spectrum_msg.duration.nanosec = int((spectrum.duration.total_seconds() - spectrum_msg.duration.sec) * 1e9)  # 10 seconds duration
        spectrum_msg.a0 = spectrum.a0
        spectrum_msg.a1 = spectrum.a1
        spectrum_msg.a2 = spectrum.a2
        spectrum_msg.counts = spectrum.counts
        self.spectrum_publisher_.publish(spectrum_msg)

        databuf = self.rc.data_buf()

        last = None
        for v in databuf:
            if isinstance(v, RealTimeData):
                if last is None or last.dt < v.dt:
                    last = v

        if last is None:
            return []

        # ts = int(last.dt.timestamp())

        count_rate_msg.data = last.count_rate # CPS
        self.count_rate_publisher_.publish(count_rate_msg)

        dose_rate_msg.data = (10_000.00 * last.dose_rate) # μSv/h
        self.dose_rate_publisher_.publish(dose_rate_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadiacodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()