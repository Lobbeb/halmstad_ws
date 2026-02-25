import os
import sys
import time
import uuid
import math

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import quaternion_from_euler, quaternion_multiply
from ros_gz_interfaces.srv import SetEntityPose

class GzCommand(Node):
    def __init__(self):    
        super().__init__('gz_command')
        self.group = ReentrantCallbackGroup()
        self.declare_parameter("command", "setpose")
        self.declare_parameter("world", "orchard")
        self.declare_parameter("name", "dji0")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 10.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("pitch", -45.0)

        self.z_offset = 0.27
        
        self.command = self.get_parameter("command").value
        self.world = self.get_parameter("world").value
        self.name = self.get_parameter("name").value
        self.x = self.get_parameter("x").value
        self.y = self.get_parameter("y").value
        self.z = self.get_parameter("z").value
        self.pitch = self.get_parameter("pitch").value
        self.yaw = self.get_parameter("yaw").value

        self.init_scan_flag = True
        self.cli = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose', callback_group=self.group)
        max_wait_s = 10.0
        waited = 0.0
        print(f"WAIT for service: /world/{self.world}/set_pose")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            waited += 1.0
            self.get_logger().info(f"service not available, waiting... ({waited:.0f}/{max_wait_s:.0f}s)")
            if waited >= max_wait_s:
                raise RuntimeError(f"Service /world/{self.world}/set_pose not available after {max_wait_s:.0f}s")
        self.timer = self.create_timer(0.1, self.do_command, callback_group=self.group)

    def set_pose(self, name, x, y, z, yaw_deg):
        try:
            robot_request = SetEntityPose.Request()
            quat1 = quaternion_from_euler(0.0, 0.0, math.radians(self.yaw))
            robot_request.entity.id = 0
            robot_request.entity.name = name
            robot_request.entity.type = robot_request.entity.MODEL
            robot_request.pose.position.x = x
            robot_request.pose.position.y = y
            robot_request.pose.position.z = z
            robot_request.pose.orientation.x = quat1[0]
            robot_request.pose.orientation.y = quat1[1]
            robot_request.pose.orientation.z = quat1[2]
            robot_request.pose.orientation.w = quat1[3]
            return self.cli.call_async(robot_request)
        except Exception as ex:
            print("Exception set_pose:", ex, type(ex))

    def set_camera(self, name, x, y, z, pitch):
        try:
            camera_request = SetEntityPose.Request()
            quat2 = quaternion_from_euler(0.0, -math.radians(self.pitch), 0.0)
            camera_request.entity.id = 0
            camera_request.entity.name = name
            camera_request.entity.type = camera_request.entity.MODEL
            camera_request.pose.position.x = x
            camera_request.pose.position.y = y
            camera_request.pose.position.z = z-self.z_offset
            camera_request.pose.orientation.x = quat2[0]
            camera_request.pose.orientation.y = quat2[1]
            camera_request.pose.orientation.z = quat2[2]
            camera_request.pose.orientation.w = quat2[3]
            return self.cli.call_async(camera_request)
        except Exception as ex:
            print("Exception set_camera:", ex, type(ex))

    def init_scan(self, x, y, z, dx, dy, nx, speed):
        try:
            print("init_scan:", x, y, z, dx, dy, nx, speed)
            self.scan_x0 = x
            self.scan_y0 = y
            self.scan_x = x-dx
            self.scan_y = y-dy
            self.scan_z = z
            self.scan_dx = dx
            self.scan_dy = dy
            self.scan_nx = nx
            self.scan_speed = speed
            self.scan_cdx = 0.0
            self.scan_cdy = 1.0
            self.scan_index = 0
            self.init_scan_flag = False
            self.scan_move_x = 0.0
        except Exception as ex:
            print("Exception init_scan:", ex, type(ex))

    def update_scan(self, time):
        try:
            self.scan_x += time*self.scan_speed*self.scan_cdx
            self.scan_y += time*self.scan_speed*self.scan_cdy
            if self.scan_y > self.scan_y0+self.scan_dy and self.scan_cdy > 0.5:
                print("MOVE POSITIVE Y:", self.scan_move_x)            
                self.scan_cdy = 0.0
                self.scan_cdx = 1.0
                self.scan_move_x = 2*self.scan_dx/self.scan_nx
                return
            if self.scan_y < self.scan_y0-self.scan_dy and self.scan_cdy < -0.5:
                print("MOVE NEGATIVE Y:", self.scan_move_x)
                self.scan_cdy = 0.0
                self.scan_cdx = 1.0
                self.scan_move_x = 2*self.scan_dx/self.scan_nx
                return
            if self.scan_cdx > 0.5:
                # print("MOVE X:", self.scan_move_x)            
                dx = time*self.scan_speed
                self.scan_x += dx
                self.scan_move_x -= dx
                if self.scan_move_x < 0.0:
                    self.scan_cdx = 0.0
                    self.scan_index += 1
                    if self.scan_index % 2:
                        self.scan_cdy = -1.0
                    else:
                        self.scan_cdy = 1.0
            if self.scan_index >= self.scan_nx:
                print("CANCEL SCAN")
                self.timer.cancel()
        except Exception as ex:
            print("Exception update_scan:", ex, type(ex))
    
    def do_command(self):
        try:
            # print("do_command:", self.command, self.name)
            if self.command == "setpose":
                self.timer.cancel()
                camera_name = self.name + "_camera0"
                print(self.x, self.y, self.z)
                print(self.yaw, self.pitch)

                f1 = self.set_pose(self.name, self.x, self.y, self.z, self.yaw)
                f2 = self.set_camera(camera_name, self.x, self.y, self.z, self.pitch)

                # Wait briefly for responses (bounded so we never hang)
                t0 = time.time()
                timeout_s = 2.0
                while time.time() - t0 < timeout_s:
                    if f1.done() and f2.done():
                        break
                    time.sleep(0.01)

                # Exit the node cleanly
                rclpy.shutdown()
                return
            
            if self.command == "scan":
                dx = 10.0
                dy = 10.0
                nx = 7
                if self.init_scan_flag:
                    self.init_scan(self.x, self.y, self.z, dx, dy, nx, 3.0)
                self.update_scan(0.1)
                self.set_pose(self.name, self.scan_x, self.scan_y, self.scan_z, 0.0)
                camera_name = self.name + "_camera0"                
                self.set_camera(camera_name, self.scan_x, self.scan_y, self.scan_z, self.pitch)
                
        except Exception as ex:
            print("Exception:", ex, type(ex))

def main(args=None):
    rclpy.init(args=args)
    node = GzCommand()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    print("COMMAND:", node.command)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    executor.shutdown()    
    if rclpy.ok():
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    
