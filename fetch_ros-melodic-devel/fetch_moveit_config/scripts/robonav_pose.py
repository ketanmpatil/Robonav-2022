#! /usr/bin/env python
import tf2_ros
import sys
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

# INITIALIZATION OF COORDINATES 
x = 0
y = 0
z = 0

feedback = String()

rospy.init_node('node_pose_end_effector', anonymous=True) # ROS Node Initialization
commander = moveit_commander.roscpp_initialize(sys.argv) # Moveit Commander Initialization
robot = moveit_commander.RobotCommander() # Robot Commander

message = None
shape = None

def msg_callback(data):
    global message
    message = data.data

def shape_callback(data):
    global shape
    shape = data.data

def end_eff_pose(x, y, z, planning_group):
    ''' Takes coordinates and name of the planning group as an input and commands the end to 
    that position. '''
    move_group = moveit_commander.MoveGroupCommander(planning_group)

    move_group.allow_replanning(False)

   

    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = 'base_link'

    # DEFAULT ORIENTATION, CALCULATED FROM MANUAL METHODS 

    roll= 3.07
    pitch= 0.48 #-0.0115756034919
    yaw= -0.164 #0.0272472345585


    # TO CONVERT EULER COORDINATES TO QUATERNION 
    quaternion = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
    pose_goal.pose.orientation.x=quaternion[0]
    pose_goal.pose.orientation.y=quaternion[1]
    pose_goal.pose.orientation.z=quaternion[2]
    pose_goal.pose.orientation.w=quaternion[3]
	 

    pose_goal.pose.position.x = round(x, 6)-0.17
    pose_goal.pose.position.y = round(y, 6) # -0.17 IS THE OFFSET FROM ENDPOSE TO GRIPPERS
    pose_goal.pose.position.z = round(z, 6)

    try:
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        move_group.execute(plan)
    except:
        raise

def go_to_predefined(planning_group, arg_pose_name):
    '''
    Function defined to move the arm to a predefined pose.
    It takes 2 argument, 
    planning_group: Name of Planning Group
    arg_pose_name: name of predefined pose
    '''
    group = moveit_commander.MoveGroupCommander(planning_group)

    exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    exectute_trajectory_client.wait_for_server()

    rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
    group.set_named_target(arg_pose_name)
    group.go(wait = True)
    rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

def add_scene_obj(x,y,z):
    ''' Add the scene objects, for the given transformations.
        takes the mesh from local folders.
    '''

    rospy.sleep(1)

    scene = moveit_commander.PlanningSceneInterface(synchronous = True)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # above the panda_hand frame
    box_name = "sphere"

    scene.add_box(box_name, box_pose,size=(0.07,0.07,0.07))

    rospy.loginfo('Added Object')

def get_transforms(frame, tf_buffer, rate):
    '''
    Function used to return transforms for the tomato.
    '''
    global x, y, z

    try:
            # transform = tf_buffer.lookup_transform('camera_depth_frame2', 'obj1','base_link', rospy.Time(0), rospy.Duration(2.0)) 
            transform = tf_buffer.lookup_transform('base_link', frame, rospy.Time(0), rospy.Duration(3.0)) 
            print('Done Listening. Transforms Received!')
            
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            raise

    x = transform.transform.translation.x
    y = transform.transform.translation.y
    z = transform.transform.translation.z

def main():

    tf_buffer = tf2_ros.Buffer()
    listener =tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(5) # 5Hz
    group = moveit_commander.MoveGroupCommander('arm')
    
    pub = rospy.Publisher('chatter', String, queue_size=10)

    rospy.Subscriber('chatter', String, msg_callback)
    rospy.Subscriber('shape', String, shape_callback)

    while not rospy.is_shutdown():

        if message == 'execute':
            go_to_predefined('head', 'view')
            go_to_predefined('arm', 'catch')
            get_transforms('shape', tf_buffer, rate)
            add_scene_obj(x,y,z)
            end_eff_pose(x, y,z, 'arm')
            go_to_predefined('gripper', 'close')
            feedback.data = 'drop'
            pub.publish(feedback)
            rate.sleep()
            break
        
        elif message == 'DROP':
            SHAPE = 'Shape: ' + str(shape)
            rospy.loginfo(SHAPE)
            go_to_predefined('arm', 'drop')
            go_to_predefined('gripper', 'open')
            feedback.data = 'next_row'
            pub.publish(feedback)
            rate.sleep()
            break


    moveit_commander.roscpp_shutdown()

    

if __name__ == '__main__':
    main()

