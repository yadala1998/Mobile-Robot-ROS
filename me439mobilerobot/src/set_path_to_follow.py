#!/usr/bin/env python
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439WheelSpeeds")
from me439mobilerobot.msg import ME439PathSpecs
from std_msgs.msg import Bool


# =============================================================================
#     Set up a time course of commands
# =============================================================================

# =============================================================================
# # NEW: Determine paths by lines, arcs, pivots and pauses, and create a 
# #  "robot" object to plan it for you. 
# =============================================================================

# Get parameters from rosparam
wheel_width = rospy.get_param('/wheel_width_model') # All you have when planning is a model - you never quite know the truth! 
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0

# Create a mobile robot object from the Imported module "me439_mobile_robot_class"
# REMEMBER to call the right file (i.e., use the _HWK if needed)
import me439_mobile_robot_class_v01 as m439rbt
robot = m439rbt.robot(wheel_width, body_length, wheel_radius)

#%%
###############################################################################
# PATH_SPECS for a Closed-Loop path-following controller.
# Each segment is defined as an Arc with parameters: 
# (Xorigin, Yorigin, Theta_init, Rsigned, Length)
#-   Xorigin and Yorigin are in the World frame
#-   Theta_init is in the World frame, measuring the angle of the intended ROBOT FORWARD 
#     direction as a +z rotation from the +Yworld axis
#-   Rsigned should be + for Left turn, - for Right turn, or +/- infinity (np.inf (numpy.inf)) for a Line
#-   Length is a path Arclength for this segment. (for a Line, the line's length)
###############################################################################
# SEVERAL EXAMPLES of Direct path programming: 
# the last one active will be executed. 
###############################################################################
## 1 m circle starting at [0.5,0.5]
#path_specs = np.array([[0.5,0.5,0,1,2*np.pi*1]])
## 1 m circle starting at [-2,-2]
#path_specs = np.array([[-2,-2,0,1,2*np.pi*1]])
## Picture of a cart with two wheels
#path_specs = np.array([[0,0,0,np.inf,1],[0,1,-np.pi/2,np.inf,3], [3,1,-np.pi,np.inf,1], [3,0,np.pi/2,np.inf,0.5],[2.5,0,np.pi/2,0.5,2*np.pi*0.5],[2.5,0,np.pi/2,np.inf,2],[0.5,0,np.pi/2,0.5,2*np.pi*0.5], [0.5,0,np.pi/2,np.inf,0.5]])

###############################################################################
# ALTERNATIVELY, use Geometric Constructors to automatically make the specs: 
#   specify_line(x0,y0,xf,yf)
#   specify_arc(x0,y0,xf,yf,R,way='short')     # or way='long' (do you want the "short way" or the "long way" around the circle?)
###############################################################################
# 1 m circle starting at [0.5,0.5], using geometric constructor functions. 
# NOTE that a full circle is a degenerate case, so it has to be done in two halves: 
path_specs = np.array([robot.specify_arc(0.5,0.5,2.5,0.5,-1,way='long'),robot.specify_arc(2.5,0.5,0.5,0.5,-1,way='long')] )
## 1 m circle starting at [-2,-2], using geometric constructor functions. 
## NOTE that a full circle is a degenerate case, so it has to be done in two halves: 
#path_specs = np.array([robot.specify_arc(-2,-2,0,-2,-1,way='long'),robot.specify_arc(0,-2,-2,-2,-1,way='long')] )

####  SIMILAR for other Paths of your own design ####

## Interesting BUG demos: 
## EXPLAIN what's wrong with these examples: 
## line from [0,-1] to [0,-0.1] 
## ** STUDENTS: THINK ABOUT WHY THIS BEHAVES ODDLY
#path_specs = np.array([[0,-1,0,np.inf,0.9]])
## Line from [-1,3] to [3,3]:
#path_specs = np.array([[-1,3,-np.pi/2,np.inf,5]])
## Line from [-1,-1] to [-1,3]:
#path_specs = np.array([[-1,-1,0,np.inf,4]])


###############################################################################
# FIX THE PATH_SPECS to drive a line from the initial position to 
# the path starting point, before beginning the specified path 
# (if not already there): 
if not all( path_specs[0,0:2] == robot.r_center_world) :
    path_specs = np.append(np.array([robot.specify_line(robot.r_center_world[0], robot.r_center_world[1],path_specs[0,0],path_specs[0,1])]),path_specs,axis=0)
###############################################################################

##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you could skip segments if you wanted to)
segment_number = 0  # for Segmented path following. 
path_complete = Bool()
# =============================================================================
# # END of new section onplanning with line and arcs
# =============================================================================


# Publish desired wheel speeds at the appropriate time. 
def talker(): 
    global path_specs, segment_number, path_complete
    # Launch a node called "set_path_to_follow"
    rospy.init_node('set_path_to_follow', anonymous=False)

    # Create the publisher. Name the topic "path_segment_spec", with message type "ME439PathSpecs"
    pub_segment_specs = rospy.Publisher('/path_segment_spec', ME439PathSpecs, queue_size=1)
    
    # Create the publisher. Name the topic "path_complete", with message type "Bool"
    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    # Create a publisher that listens for messages on the "segment_complete" topic
    sub_complete = rospy.Subscriber('/segment_complete', Bool, increment_segment)
    
    # Declare the message to publish. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away using . 
    path_segment_spec = ME439PathSpecs()
   

    
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz
    try: 
        # start a loop 
        while not rospy.is_shutdown():
            path_segment_spec.x0 = path_specs[segment_number,0]
            path_segment_spec.y0 = path_specs[segment_number,1]
            path_segment_spec.theta0 = path_specs[segment_number,2]
            path_segment_spec.Radius = path_specs[segment_number,3]
            path_segment_spec.Length = path_specs[segment_number,4]
            # Actually publish the message
            pub_segment_specs.publish(path_segment_spec)
            # Log the info (optional)
#            rospy.loginfo(pub_speeds)    
            
            
            pub_path_complete.publish(path_complete)
            
            r.sleep()
#        
#        # Here step through the settings. 
#        for stage in range(0,len(stage_settings_array)):  # len gets the length of the array (here the number of rows)
#            # Set the desired speeds
#            dur = stage_settings_array[stage,0]
#            msg_out.v_left = stage_settings_array[stage,1]
#            msg_out.v_right = stage_settings_array[stage,2]
#            # Actually publish the message
#            pub_speeds.publish(msg_out)
#            # Log the info (optional)
#            rospy.loginfo(pub_speeds)       
#            
#            # Sleep for just long enough to reach the next command. 
#            sleep_dur = dur - (rospy.get_rostime()-t_start).to_sec()
#            sleep_dur = max(sleep_dur, 0.)  # In case there was a setting with zero duration, we will get a small negative time. Don't use negative time. 
#            rospy.sleep(sleep_dur)
#        
    except Exception:
        traceback.print_exc()
        pass
        
        
        


def increment_segment(msg_in):
    # get access to the globals set at the top
    global segment_number, path_complete

    segment_number = segment_number + 1
    
    if segment_number >= path_specs.shape[0]:
        path_complete.data = True
        segment_number = segment_number - 1
    else:
        path_complete.data = False
    



if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
