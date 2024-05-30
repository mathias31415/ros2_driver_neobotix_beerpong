# -*- coding: utf-8 -*-
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from moveit_wrapper.srv import MoveToPose, MoveToJointPosition, SetVelocity, String  #import the custom service interfaces from the wrapper package
from geometry_msgs.msg import Pose as PoseMsg
from manipulation_tasks.transform import Affine
from geometry_msgs.msg import Quaternion, Point
from std_srvs.srv import SetBool, Trigger    #change in SetBool for our gripper driver
import copy

from ros_environment.lib.base_node import BaseNode  #import the get_transform method to get affine transforms between X and world
from .util import affine_to_pose

#imports for ur specific io-control
from ur_msgs.srv import SetIO


# TODO use manipulation_tasks protocol for Robot


class RobotClient:  #this is the class which gets called in your application// when calling the init method the clients for the services in moveit_wrapper_node get initialized
    """ 
    TODO description
    """

    def __init__(self, node: Node = None, is_simulation: bool = False) -> None:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        if node is None:
            self.node = BaseNode("robot_client", is_simulation) #starts the BaseNode (lib folder) --> get_transform method
        else:
            self.node = node

        self.move_lin_cli = self.node.create_client(MoveToPose, "/move_to_pose_lin")    #connects to the services defined in the moveit_wrapper srvs
        while not self.move_lin_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move_to_pose_lin service not available, waiting some more ...")
        self.node.get_logger().info("move_to_pose_lin service available")

        self.move_ptp_cli = self.node.create_client(MoveToPose, "/move_to_pose_ptp")
        while not self.move_ptp_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move_to_pose_ptp service not available, waiting some more ...")
        self.node.get_logger().info("move_to_pose_ptp service available")

        self.move_joint_cli = self.node.create_client(MoveToJointPosition, "/move_to_joint_position")
        while not self.move_joint_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move_to_joint_position service not available, waiting some more ...")
        self.node.get_logger().info("move_to_joint_position service available")

        self.reset_planning_group_cli = self.node.create_client(String, "/reset_planning_group")
        while not self.reset_planning_group_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("reset_planning_group service not available, waiting some more ...")
        self.node.get_logger().info("reset_planning_group service available")

        self.set_velocity_cli = self.node.create_client(SetVelocity, "/setVelocityScaling")
        while not self.set_velocity_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("setVelocityScaling service not available, waiting some more ...")
        self.node.get_logger().info("setVelocityScaling service available")
 
        #Instanciate gripper client
        self.gripper_cli = self.node.create_client(SetIO, "/io_and_status_controller/set_io")       #bool service 0 open/ 1 close instead of 2 diffrent trigger services
        while not self.gripper_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("set_io service for gripper not available, waiting again...")
        self.node.get_logger().info("set_io service for gripper available")


        

        # default for neobotix (no collision) -> shuold be overwritten by application
        self.home_position = [-np.pi/2, -np.pi/2,-np.pi/2, 3*np.pi/2, np.pi/2, 0.0]


    # this are the methods which can be called in your application which are communicating to the robot
    def home(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        return self.ptp_joint(self.home_position)  # and gripper_success

    def ptp(self, pose: Affine) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = RobotClient.send_request(req, self.move_ptp_cli)
        response = self.wait_for_response(future)
        return response.success

    def ptp_joint(self, joint_positions: List[float]) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.
        :param joint_positions:

        """
        req = MoveToJointPosition.Request()
        req.joint_position = joint_positions
        future = RobotClient.send_request(req, self.move_joint_cli)
        response = self.wait_for_response(future)
        return response.success

    def lin(self, pose: Affine) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = RobotClient.send_request(req, self.move_lin_cli)
        response = self.wait_for_response(future)
        return response.success

    def reset_planning_group(self, planning_group) -> bool:
        req = String.Request()
        req.data = planning_group
        future = RobotClient.send_request(req, self.reset_planning_group_cli)
        response = self.wait_for_response(future)
        return response.success


    #Gripper driver
    def move_gripper(self, request = 0.0) -> bool:    #pass 0.0 for open and 1.0 for close
        """
        TODO docstring

        Parameters
        ----------
        int8: fun         "set function to perform: set D-out = 1"
        int8: pin         "set pin to use: CONF-out 2 = 10"
        float32: state    "set signal level: STATE_ON = 1, STATE_OFF = 0 for D-out"

        Returns
        -------
        bool success

        """
        s = True
 
        req = SetIO.Request() # init the message object
        req.fun = 1     # D-out  
        req.pin = 10    # Conf-out 2  
        req.state = request  #pass the given argument to the server node

        future = RobotClient.send_request(req, self.gripper_cli)
        response = self.wait_for_response(future)
        s = response.success
        if not s:
            self.node.get_logger().info("moving the gripper failed.")
        return s
    

    def setVelocity(self, fraction) -> bool:
        req = SetVelocity.Request()
        req.velocity_scaling = fraction
        future = RobotClient.send_request(req, self.set_velocity_cli)
        response = self.wait_for_response(future)
        return response.success
        


    @staticmethod
    def send_request(request, client):
        future = client.call_async(request)
        return future

    def wait_for_response(self, future):
        """
        TODO docstring

        Parameters
        ----------
        future : TYPE
            DESCRIPTION.

        Returns
        -------
        response : TYPE
            DESCRIPTION.

        """
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response



    def destroy_node(self) -> None:
        self.node.destroy_node()
