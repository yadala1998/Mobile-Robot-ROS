#!/usr/bin/env python
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439WaypointXY")
from me439mobilerobot.msg import ME439WaypointXY
from std_msgs.msg import Bool


# =============================================================================
# # Set waypoints to hit along the path 
# =============================================================================
# Get parameters from rosparam
## none needed for waypoint setting ##

# Waypoints to hit: a "numpy.array" of [x,y] coordinates. 
# Example: Square
waypoints = np.array([[0.5, 0.],[0.5,0.5],[0.,0.5],[0.,0.]])

# =============================================================================
# # END of section on waypoint setting
# =============================================================================


##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you could skip segments if you wanted to)
waypoint_number = 0  # for waypoint seeking. 
path_complete = Bool()

# Create the publisher for the topic "waypoint_xy", with message type "ME439WaypointXY"
pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)

# Create the publisher for the topic "path_complete", with message type "Bool"
pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)



# Publish desired waypoints at the appropriate time. 
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_waypoint, pub_path_complete
    # Launch a node called "set_waypoints"
    rospy.init_node('set_waypoints', anonymous=False)
 
    
    # # Create the publisher. Name the topic "waypoint_xy", with message type "ME439WaypointXY"
    # pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)
    
    # # Create the publisher. Name the topic "path_complete", with message type "Bool"
    # pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    # Create a subscriber that listens for messages on the "waypoint_complete" topic
    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)
    
    # Declare the message to publish. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away using . 
    msg_waypoint = ME439WaypointXY()
   

    
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz
    try: 
        # start a loop 
        while not rospy.is_shutdown():
            msg_waypoint.x = waypoints[waypoint_number,0]
            msg_waypoint.y = waypoints[waypoint_number,1]
            
            # Actually publish the message
            pub_waypoint_xy.publish(msg_waypoint)
            # Log the info (optional)
#            rospy.loginfo(msg_waypoint)    
            
            pub_path_complete.publish(path_complete)
            
            r.sleep()

    except Exception:
        traceback.print_exc()
        pass
        
        
        


def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete
    
    if msg_in.data:  # it's Boolean
        waypoint_number = waypoint_number + 1
    
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
        # waypoint_number = waypoint_number - 1  # This line prevents an array out of bounds error to make sure the node stayed alive. By commenting, allow it to increment past the end, which will throw an exception (array out of bounds) the next time it publishes a waypoint and cause the node to die. 
    else:
        path_complete.data = False
    
    pub_path_complete.publish(path_complete)



if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
