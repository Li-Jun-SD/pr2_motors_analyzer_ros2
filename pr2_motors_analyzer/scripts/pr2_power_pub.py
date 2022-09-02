#!/usr/bin/env python3

from array import array
import rclpy
from rclpy.node import Node
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('pr2_motor_power_sim')
    pub = node.create_publisher(DiagnosticArray, '/diagnostics', 1)
    logger = node.get_logger()
    
    my_rate = node.create_rate(1.0)

    # If you don't create a thread, in while loop would only run once and never break.
    # I don't know why.
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    array = DiagnosticArray()
    # Fake power board status, estop is on
    power_stat = DiagnosticStatus(name = 'Power board 1000', level = DiagnosticStatus.OK, message = 'Running', hardware_id = "motors")
    power_stat.values = [ KeyValue(key = 'Runstop hit', value = 'False'), KeyValue(key = 'Estop hit', value = 'False') ]
    # Fake EtherCAT Master status, all OK
    eth_stat = DiagnosticStatus(name='EtherCAT Master', level = DiagnosticStatus.OK, message = 'OK')

    array.status = [ power_stat, eth_stat ]
    array.header.frame_id = "PR2"
    array.header.stamp = node.get_clock().now().to_msg()

    try:
        while rclpy.ok():
            logger.info("nihao")
            pub.publish(array)
            my_rate.sleep()
    except KeyboardInterrupt:
        logger.info("ctrl-C to interrupt.")
        pass

    rclpy.shutdown()
    thread.join()
