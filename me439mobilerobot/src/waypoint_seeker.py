#!/usr/bin/env python
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message types by name (
from me439mobilerobot.msg import ME439WaypointXY, ME439PathSpecs 
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool


waypoint_tolerance = rospy.get_param('/waypoint_tolerance') # how close to a way point must it be to call it good? 

# global variable to hold the waypoint currently being tracked
waypoint = ME439WaypointXY()
waypoint.x = np.nan     # Set to Not A Number initially so it does not think the job is finished. 
waypoint.y = np.nan

# Global to track the state of completion of the  waypoint    
waypoint_complete = Bool()
waypoint_complete.data = False    

# =============================================================================
#     Set up a waypoint seeker
#     Subscribe to "robot_pose_estimated" (Pose2D)
#     and to "waypoint_xy"  (ME439WaypointXY)
#     Publish "path_segment_spec" (ME439PathSpecs)
# =============================================================================

# Get parameters from rosparam
## none needed for waypoint seeker ##

# Create the publisher. Name the topic "path_segment_spec", with message type "ME439PathSpecs"
pub_segment_specs = rospy.Publisher('/path_segment_spec', ME439PathSpecs, queue_size=1)

# Create the publisher for the topic "waypoint_complete", with message type "Bool"
pub_waypoint_complete = rospy.Publisher('/waypoint_complete', Bool, queue_size=1)


# Publish desired wheel speeds at the appropriate time. 
def talker(): 
    # Actually launch a node called "waypoint_seeker"
    rospy.init_node('waypoint_seeker', anonymous=False)
    
    # Create a Subscriber to the robot's current estimated position
    # with a callback to "set_path_to_waypoint"
    sub_robot_pose_estimated = rospy.Subscriber('/robot_pose_estimated', Pose2D, set_path_to_waypoint)
    
    # Subscriber to the "waypoint_xy" topic
    sub_waypoint = rospy.Subscriber('/waypoint_xy', ME439WaypointXY, set_waypoint)
      
    # Prevent the node from exiting
    rospy.spin()    



# =============================================================================
# # Function to update the path to the waypoint based on the robot's estimated position
# =============================================================================
def set_path_to_waypoint(pose_msg_in):
    # First assign the incoming message
    global estimated_pose, waypoint, pub_segment_specs, waypoint_complete
    estimated_pose = pose_msg_in
        
    # set the path to be directly from here to the waypoint
    dx = waypoint.x - estimated_pose.x     # change in X coords
    dy = waypoint.y - estimated_pose.y  # change in Y coords
    
    # Compute the ME439PathSpecs message
    path_segment_spec = ME439PathSpecs()
    path_segment_spec.x0 = estimated_pose.x
    path_segment_spec.y0 = estimated_pose.y
    path_segment_spec.theta0 = np.arctan2(-dx, dy)
    path_segment_spec.Radius = np.inf
    path_segment_spec.Length = np.sqrt(dx**2 + dy**2)
    
    
    #  Publish it
    pub_segment_specs.publish(path_segment_spec)
    
    if (path_segment_spec.Length < waypoint_tolerance) and not waypoint_complete.data: # DO NOT send more than once. Wait for a new path segment before sending "waypoint_complete" again 
        waypoint_complete.data = True
        pub_waypoint_complete.publish(waypoint_complete)
#        waypoint_complete.data = False # set back to False
#    else:
#        waypoint_complete.data = False
#        pass
    

# Function to receive a Waypoint and set the goal point to it.     
def set_waypoint(waypoint_msg_in): 
    global waypoint, waypoint_complete
    waypoint = waypoint_msg_in
    waypoint_complete.data = False
#    pub_waypoint_complete.publish(waypoint_complete)

        
    

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
