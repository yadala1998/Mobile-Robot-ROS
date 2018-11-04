#!/usr/bin/env python

# =============================================================================
# 2018-10-12
# =============================================================================


import rospy
import numpy as np
import traceback 
from geometry_msgs.msg import Pose2D
from me439mobilerobot.msg import ME439WheelDisplacements

#==============================================================================
# # Get parameters from rosparam
# # NOTE this is the Estimator, so we should use the "model" parameters. 
# # This will enable us to compare the "simulated" robot (considered the true robot location) 
# #  and the "estimated" robot (position estimated based on dead-reckoning)
#==============================================================================
wheel_width = rospy.get_param('/wheel_width_model')
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0


# Global variables for the robot's position
r_center_world_estimated = np.array([0.,0.])
theta_estimated = 0.
# Global variables for the robot's wheel displacements (for memory)
d_left_previous = 0.
d_right_previous = 0.

# Rate to set how often the estimated "pose" is published
f = 10.     # Hz 


def listener(): 
    global r_center_world_estimated, theta_estimated
    
    
# =============================================================================
#     # Launch a node called "mobile_robot_simulator"
# =============================================================================
    rospy.init_node('dead_reckoning', anonymous=False)
    
#==============================================================================
#     # Here start a Subscriber to the "/robot_wheel_displacements" topic.
#==============================================================================
    sub_wheel_disps = rospy.Subscriber('/robot_wheel_displacements', ME439WheelDisplacements, dead_reckoning)  
    
#==============================================================================
#     # Here start a Subscriber to the "/robot_position_override" topic.
#==============================================================================
    sub_position_override = rospy.Subscriber('/robot_position_override', Pose2D, set_pose)  
    
    
#==============================================================================
#     # Here a Publisher for the Estimated Robot Pose. 
#     # Topic '/robot_pose_estimated', Message type: Pose2D
#==============================================================================
    pub_robot_pose_estimated = rospy.Publisher('/robot_pose_estimated', Pose2D, queue_size = 1)
    robot_pose_estimated_message = Pose2D()

    
    # Rate object to set a publication rate
    r = rospy.Rate(f)
    
# =============================================================================
#     # Loop to run the publication
# =============================================================================
    while not rospy.is_shutdown():
        # Publish the pose
        robot_pose_estimated_message.x = r_center_world_estimated[0]
        robot_pose_estimated_message.y = r_center_world_estimated[1]
        robot_pose_estimated_message.theta = theta_estimated
        pub_robot_pose_estimated.publish(robot_pose_estimated_message)

        
        rospy.loginfo(pub_robot_pose_estimated)
        
        r.sleep()
        
# =============================================================================
# # Callback function for "dead-reckoning" (alternatively called "odometry")
# =============================================================================
def dead_reckoning(msg_in): 
    # These global variables hold the estimated robot state
    global r_center_world_estimated, theta_estimated, wheel_width
    # More globals to store the previous values of the wheel displacements    
    global d_left_previous, d_right_previous
    
    
#    CODE HERE: extract the wheel displacements from the message file. 
    # Look in the message file for ME439WheelDisplacements.msg to find the variable names for left and right wheel displacements. 
    # Or with "roscore" running, just ask ROS: "rosmsg show ME439WheelDisplacements"
    d_left = msg_in.d_left
    d_right = msg_in.d_right
    
#    CODE HERE: Compute the CHANGE in displacement of each wheel
    diff_left = d_left - d_left_previous
    diff_right = d_right - d_right_previous
    
#    STORE the new values of d_left and d_right for the next call
    d_left_previous = d_left
    d_right_previous = d_right
    
#    CODE HERE: compute change in path length and change in angle
    diff_pathlength = (diff_left+diff_right)/2
    diff_theta = (diff_right-diff_left)/wheel_width

#    CODE HERE: compute the average heading angle during the movement    
    theta_avg = theta_estimated + diff_theta/2.
#    Code here: compute the change in position and heading according to the dead-reckoning equations
    r_center_world_estimated[0] += diff_pathlength * -np.sin(theta_avg)
    r_center_world_estimated[1] += diff_pathlength * np.cos(theta_avg)
    theta_estimated += diff_theta

#==============================================================================
#     End of function "dead_reckoning"
#==============================================================================
    
# =============================================================================
# # Callback function for "set_pose" (to override the dead_reckoning position)
# =============================================================================
def set_pose(msg_in): 
    # These global variables will be useful. 
    global r_center_world_estimated, theta_estimated
    
#    CODE HERE: extract the pose from the message file. 
    # Look in the message file for ME439WheelDisplacements.msg to find the variable names for left and right wheel displacements. 
    # Or with "roscore" running, just ask ROS: "rosmsg show ME439WheelDisplacements"
    r_center_world_estimated[0] = msg_in.x
    r_center_world_estimated[1] = msg_in.y
    theta_estimated = msg_in.theta
        
#==============================================================================
#     # End of function "set_pose"
#==============================================================================
    
    
if __name__ == '__main__':
    try: 
        listener()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
