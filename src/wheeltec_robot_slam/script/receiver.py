#!/usr/bin/env python3
import socket
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

UDP_IP = "0.0.0.0"  # 监听所有接口
UDP_PORT = 5005     # 监听的端口号
TOPIC_NAME = "/odom"

class UdpSubscriber(Node):
    def __init__(self):
        super().__init__('udp_subscriber')
        self.publisher = self.create_publisher(Odometry, TOPIC_NAME, 10)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((UDP_IP, UDP_PORT))

    def run(self):
        while rclpy.ok():
            try:
                data, addr = self.udp_socket.recvfrom(4096)
                self.process_data(data)
            except KeyboardInterrupt:
                self.get_logger().info('Keyboard interrupt, shutting down.')
                break
            except Exception as e:
                self.get_logger().error(f'Error processing UDP data: {e}')
                break

    def process_data(self, data):
        try:
            json_str = data.decode('utf-8')
            message_dict = json.loads(json_str)
            msg = self.dict_to_msg(message_dict)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing JSON data: {e}')

    def dict_to_msg(self, msg_dict):
        msg = Odometry()
        msg.header.stamp.sec = int(msg_dict['header']['stamp'])
        msg.header.stamp.nanosec = int((msg_dict['header']['stamp'] - msg.header.stamp.sec) * 1e9)
        msg.header.frame_id = msg_dict['header']['frame_id']
        msg.child_frame_id = msg_dict['child_frame_id']

        msg.pose.pose.position.x = msg_dict['pose']['position']['x']
        msg.pose.pose.position.y = msg_dict['pose']['position']['y']
        msg.pose.pose.position.z = msg_dict['pose']['position']['z']
        msg.pose.pose.orientation.x = msg_dict['pose']['orientation']['x']
        msg.pose.pose.orientation.y = msg_dict['pose']['orientation']['y']
        msg.pose.pose.orientation.z = msg_dict['pose']['orientation']['z']
        msg.pose.pose.orientation.w = msg_dict['pose']['orientation']['w']
        msg.pose.covariance = msg_dict['pose']['covariance']

        msg.twist.twist.linear.x = msg_dict['twist']['linear']['x']
        msg.twist.twist.linear.y = msg_dict['twist']['linear']['y']
        msg.twist.twist.linear.z = msg_dict['twist']['linear']['z']
        msg.twist.twist.angular.x = msg_dict['twist']['angular']['x']
        msg.twist.twist.angular.y = msg_dict['twist']['angular']['y']
        msg.twist.twist.angular.z = msg_dict['twist']['angular']['z']
        msg.twist.covariance = msg_dict['twist']['covariance']

        return msg

def main(args=None):
    rclpy.init(args=args)
    udp_subscriber = UdpSubscriber()
    try:
        udp_subscriber.run()
    except KeyboardInterrupt:
        pass
    finally:
        udp_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
