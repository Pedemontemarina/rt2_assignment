#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation

from action_msgs.msg import GoalStatusArray

class NavInterface(Node):
    def __init__(self):
        super().__init__('nav_interface')
        
        # publisher goal
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10)
        
        # publisher cancel
        self.cancel_pub = self.create_publisher(
            Empty, 
            '/cancel_goal', 
            10)
        
        # subscriber status
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to/_action/status',
            self.status_callback,
            10)
        
        
        self.has_active_goal = False


    def status_callback(self, msg):
        TERMINAL_STATES = {4, 5, 6}  # SUCCEEDED, CANCELED, ABORTED
        for status in msg.status_list:
            if status.status in TERMINAL_STATES:
                if self.has_active_goal:
                    self.get_logger().info('Goal reached!')
                self.has_active_goal = False


    def send_goal(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        # convertiamo theta in quaternione
        
        q = Rotation.from_euler('z', theta, degrees=True).as_quat()
        # q = [x, y, z, w]
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
                
        self.goal_pub.publish(msg)
        self.has_active_goal = True
        self.get_logger().info(f'Goal sent: x={x}, y={y}, theta={theta}')

    def cancel_goal(self):
        self.cancel_pub.publish(Empty())
        self.has_active_goal = False
        self.get_logger().info('Cancel sent')

def main():
    rclpy.init()
    node = NavInterface()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    while True:
        print("\n Robot Navigation")
        print("1 - Send goal")
        print("2 - Cancel goal")

        scelta = input('What would you like to do? ').strip()

        if scelta == '1':
            # se c'è già un goal attivo chiedi
            if node.has_active_goal:
                risposta = input('There is already an active goal. Do you want to replace it? (y/n): ').strip().lower()
                if risposta != 'y':
                    print('Goal not replaced.')
                    continue
                # cancella il vecchio
                node.cancel_goal()

            try:
                x     = float(input('Insert x: '))
                y     = float(input('Insert y: '))
                theta = float(input('Insert theta (degrees): '))
                node.send_goal(x, y, theta)
            except ValueError:
                print('Error: please insert valid numbers!')

        elif scelta == '2':
            if node.has_active_goal:
                node.cancel_goal()
            else:
                print('No active goal to cancel.')

        else:
            print('Invalid choice! Please select 1, 2 or 3.')


if __name__ == '__main__':
    main()