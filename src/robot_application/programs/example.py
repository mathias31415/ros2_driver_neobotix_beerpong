import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np
import time

def movement_test(robot):
    robot.setVelocity(0.1)
    print("set velocity  to 0.1")

    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    #robot.home_position = [-0.925, -1.686, -2.074, - 1.604, -0.010, -1.682]

    robot.home_position = [-np.pi/2, -np.pi/6,-np.pi/2, 7*np.pi/6, np.pi/2, 0.0]
    print("set home position successfully")

    # move robot to home position
    robot.home()
    print("move to home position" , robot.home_position)


    # home_position = Affine((0.109, -0.077, 0.423),(0.032, 0.999, 0, 0))
    # robot.ptp(home_position)
    # print("Reached home position with ptp" , robot.camera_aruco_detection_position)

    time.sleep(5)

    camera_aruco_detection_position = Affine((0.11, 0.493, 0.285),(-0.001, 1, -0.006, 0.003))
    
    robot.lin(camera_aruco_detection_position)
    print("Reached camera pose to detect Aruco code" , camera_aruco_detection_position)

    print("open the gripper")
    robot.move_gripper(0.0)   #open
    time.sleep(2)

    print("close the gripper")
    time.sleep(2)
    robot.move_gripper(1.0)   #close
    time.sleep(2)

    print("move to home position" , robot.home_position)
    robot.setVelocity(0.1)
    robot.home()



    #######################################################################

    #print("tool0 pose in world coordinate frame" , current_pose)
    # movement_tcp = Affine((0.0, 0.4 ,0.0))
    # movement_base = Affine((0, 0.2 , 0))
    #target_pose = current_pose * movement_tcp
    #target_pose = movement_base * current_pose

    # front = Affine((0.10292, 0.38128, 0.46478),(0.0083262, 0.99996, 0.0036428, -0.0009871))
    # robot.setVelocity(0.1)
    # robot.ptp(front)
    # print("move to front" , front)

    # camera_pose_joints = [-1.75, -1.00, -1.864, - 1.604, -0.002, -1.676]
    # print("move to camera joint states")
    # robot.setVelocity(0.1)
    # robot.ptp_joint(camera_pose_joints)

    # before_pick = Affine((-0.26285, -0.31859, 0.39566),(-0.0040532, -0.016529, 0.003553, 0.99985))
    # print("move to before_pick" , before_pick)
    # robot.setVelocity(0.1)
    # robot.ptp(before_pick) 

    # print("open the gripper")
    # robot.move_gripper(0.0)   #open

    # pick = Affine((-0.2624, -0.31902, 0.57952),(-0.0040532, -0.016529, 0.003553, 0.99985))
    # print("move to pick" , pick)
    # robot.setVelocity(0.02)
    # robot.lin(pick)

    # print("close the gripper")
    # time.sleep(2)
    # robot.move_gripper(1.0)   #close
    # time.sleep(2)

    # after_pick = Affine((-0.26285, -0.31859, 0.39566),(-0.0040532, -0.016529, 0.003553, 0.99985))
    # print("move to after_pick" , after_pick)
    # robot.setVelocity(0.02)
    # robot.lin(after_pick)

    # after_pick2 = [-1.7727606932269495, -2.3591538111316126, -0.5860713163958948, -1.5373461882220667, -2.7303183714496058, -6.076037977133886]
    # print("move to after_pick2 joint states")
    # robot.setVelocity(0.05)
    # robot.ptp_joint(after_pick2)

    # front2 = Affine((0.1, 0.5, 0.5),(0.0083262, 0.99996, 0.0036428, -0.0009871))
    # print("move to front" , front2)
    # robot.setVelocity(0.1)
    # robot.ptp(front2)

    # print("move to home position" , robot.home_position)
    # robot.setVelocity(0.1)
    # robot.home()

def lin_test(robot):
    robot.setVelocity(0.1)
    print("set velocity  to 0.1")

    robot.home_position = [-np.pi/2, -np.pi/6, -np.pi/2, 7*np.pi/6, np.pi/2, 0.0]
    print("set home position successfully")

    # move robot to home position
    robot.home()
    print("move to home position" , robot.home_position)

    time.sleep(5)

    joint_pose = [-1.5707686583148401,-1.6702807585345667,-1.6521590391742151,4.893458843231201,1.5707252025604248,0.0001917476038215682]

    current_pose = robot.node.get_transform('ur_tcp_link', 'world')
    print("tcp pose in world coordinate frame" , current_pose)
    movement_tcp = Affine((0.0, 0.6 ,-0.2))
    target_pose = current_pose * movement_tcp

    print("move to test pose" , target_pose)
    robot.setVelocity(0.1)
    robot.ptp(target_pose) 
    #robot.lin(target_pose)
    #robot.ptp_joint(joint_pose)

def test_gripper (robot):
    robot.move_gripper(0.0)   #open
    print("open the gripper")

    time.sleep(5)

    robot.move_gripper(1.0)   #close
    print("open the gripper")

    time.sleep(5)

    robot.move_gripper(0.0)   #open
    print("open the gripper")

def get_current_pose(robot):
    
    current_pose = robot.node.get_transform('ur_tcp_link', 'world')
    print("tcp pose in world coordinate frame" , current_pose)
    

        
def main(args=None):
    # initialize ros communications for a given context 
    rclpy.init(args=args)

    # initialize robot client node --> this will create clients in the RobotConnection class which call services to communicate with moveit
    robot = RobotClient()     # maybe not necessary?
    
    # print("initialized Robotclient successfully")
    # robot.home_position = [-np.pi/2, -np.pi/6,-np.pi/2, 7*np.pi/6, np.pi/2, 0.0]
    # robot.home()
    # print("Reached home position :" , robot.home_position)
    # print("Compare target pose to current pose: ", robot.get_current_pose())

    
    get_current_pose(robot)
    
    movement_test(robot)
    # lin_test(robot)

    #test_gripper(robot)


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()









