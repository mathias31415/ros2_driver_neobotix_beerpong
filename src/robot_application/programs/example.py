import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np
import time

def movement_test(robot):
    robot.setVelocity(0.1)
    print("set velocity  to 0.1")

    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [-np.pi/2, -np.pi/6,-np.pi/2, 7*np.pi/6, np.pi/2, 0.0]
    print("set home position successfully")

    # move robot to home position
    robot.home()
    print("move to home position" , robot.home_position)

    time.sleep(5)

    current_pose = robot.node.get_transform('tool0', 'world')
    print("tool0 pose in world coordinate frame" , current_pose)
    movement_tcp = Affine((0, -0.2 , 0))
    movement_base = Affine((0, 0.2 , 0))


    target_pose = current_pose * movement_tcp
    #target_pose = movement_base * current_pose
    target_pose = Affine((0.18, 0.15, 0.55),(1,0,0,0))
    robot.setVelocity(0.1)
    print("set velocity  to 0.1")

    robot.lin(target_pose)
    print("move lin to pose successfully" , target_pose)

    robot.setVelocity(0.1)
    print("set velocity  to 0.1")

    robot.home()
    print("move to home position" , robot.home_position)


def test_gripper (robot):
    robot.move_gripper(0.0)   #open
    print("open the gripper")

    time.sleep(5)

    robot.move_gripper(1.0)   #close
    print("open the gripper")

    time.sleep(5)

    robot.move_gripper(0.0)   #open
    print("open the gripper")

        
def main(args=None):
    # initialize ros communications for a given context 
    rclpy.init(args=args)

    # initialize robot client node --> this will create clients in the RobotConnection class which call services to communicate with moveit
    robot = RobotClient()     # maybe not necessary?
    print("initialized Robotclient successfully")

    movement_test(robot)

    test_gripper(robot)


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()









