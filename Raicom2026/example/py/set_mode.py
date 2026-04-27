#!/usr/bin/env python3

import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import SetMcAction
from aimdk_msgs.msg import RequestHeader, CommonState, McActionCommand


class SetMcActionClient(Node):
    def __init__(self):
        super().__init__('set_mc_action_client')
        self.client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction'
        )
        self.get_logger().info('✅ SetMcAction client node created.')

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, action_name: str, source: str):
        req = SetMcAction.Request()
        req.header = RequestHeader()
        
        # 设置 source
        req.source = source 

        cmd = McActionCommand()
        cmd.action_desc = action_name
        req.command = cmd

        self.get_logger().info(
            f'📨 Sending request: Source={source}, Mode={action_name}')
        
        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break
            self.get_logger().info(f'trying ... [{i}]')

        response = future.result()
        if response is None:
            self.get_logger().error('❌ Service call failed or timed out.')
            return

        if response.response.status.value == CommonState.SUCCESS:
            self.get_logger().info('✅ Robot mode set successfully.')
        else:
            self.get_logger().error(
                f'❌ Failed to set robot mode: {response.response.message}'
            )


def main(args=None):
    action_info = {
        'PASSIVE_DEFAULT': ('PD', 'joints with zero torque'),
        'DAMPING_DEFAULT': ('DD', 'joints in damping mode'),
        'JOINT_DEFAULT': ('JD', 'Position Control Stand (joints locked)'),
        'STAND_DEFAULT': ('SD', 'Stable Stand (auto-balance)'),
        'LOCOMOTION_DEFAULT': ('LD', 'locomotion mode (walk or run)'),
        'UPPERBODY_REMOTE_SPLIT': ('US', 'UPPERBODY_REMOTE_SPLIT'),  
        'HEAD_ONLY': ('HO', 'HEAD_ONLY'),
    }

    choices = {v[0]: k for k, v in action_info.items()}

    rclpy.init(args=args)
    node = None
    try:
        # 获取参数：第一个参数是模式(motion)，第二个可选参数是来源(source)
        # 默认 source 为 'rc'
        motion = None
        source = 'rc'

        if len(sys.argv) > 1:
            motion = sys.argv[1]
            if len(sys.argv) > 2:
                source = sys.argv[2]
        else:
            print('{:<4} - {:<20} : {}'.format('abbr', 'robot mode', 'description'))
            for k, v in action_info.items():
                print(f'{v[0]:<4} - {k:<20} : {v[1]}')
            motion = input('Enter abbr of robot mode:')
            src_input = input('Enter source (default: rc): ')
            if src_input.strip():
                source = src_input

        action_name = choices.get(motion.upper())
        if not action_name:
            raise ValueError(f'Invalid abbr: {motion}')

        node = SetMcActionClient()
        node.send_request(action_name, source)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
