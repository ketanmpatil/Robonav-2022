#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String



# position : [x, y] # Format
# orientation : [X, Y, Z, W]
rows = np.array(['FIRST','SECOND','THIRD','FOURTH','FIFTH'])
carton=np.array(['FIRST','CartonF1','CartonF2','CartonF3','CartonF4','CartonF5','SECOND','CartonS1','cartonS2','cartonS3','cartonS4','cartonS5','THIRD','cartonT1','cartonT2','cartonT3','cartonT4','cartonT5','FOURTH','cartonFr1','cartonFr2','cartonFr3','cartonFr4','cartonFr5','FIFTH','cartonfi1','cartonfi2','cartonfi3','cartonfi4','cartonfi5'])
coordinates = {'FIRST' : {'position' : [18.0938037168, 13.780151332], 'orientation' : [0, 0,0.887241316603, 0.461305588643]},
    'cartonF1' : {'position' : [4.35965755979, 7.98224062358], 'orientation' : [0, 0,-0.857388931125, -0.514669040048]},
    'cartonF2' : {'position' : [2.63043927132, 7.12907187993], 'orientation' : [0, 0,-0.873795931778, -0.486292781776]},
    'cartonF3' : {'position' : [0.607793952299, 6.72419993136], 'orientation' : [0, 0,-0.415385553466, -0.909645448497]},
    'cartonF4' : {'position' : [-1.01620877819, 8.06202864399], 'orientation' : [0, 0,-0.3187153547, -0.947850474853]},
    'cartonF5' : {'position' : [-1.67359184074, 8.61039974486], 'orientation' : [0, 0,-0.871649047153, -0.490130532202]},
    'SECOND' : {'position' : [6.43308623873, -10.4516103749], 'orientation' : [0, 0,-0.215501814818, -0.976503439733]},
    'cartonS1' : {'position' : [6.0638957286, 3.43302291464], 'orientation' : [0, 0,-0.121474549565, -0.992594546533]},
    'cartonS2' : {'position' : [5.3592309345, 5.52318341517], 'orientation' : [0, 0,0.067361655462, -0.997728624113]},
    'cartonS3' : {'position' : [5.57054249131, 7.49892986771], 'orientation' : [0, 0,0.2068668753, -0.978369100035]},
    'cartonS4' : {'position' : [6.44762661408, 9.55198131967], 'orientation' : [0, 0,0.312088982588, -0.950052875869]},
    'cartonS5' : {'position' : [6.08832135737, 9.74863150594], 'orientation' : [0, 0,-0.529104497101, -0.848556675271]},
    'THIRD' : {'position' : [-4.82467647351, -7.88789982989], 'orientation' : [0, 0,0.297242055058, -0.954802157887]},
    'cartonT1' : {'position' : [6.69040506977, 0.977167704985], 'orientation' : [0, 0,0.470326326954, -0.882492575705]},
    'cartonT2' : {'position' : [8.02472172002, 2.43479355755], 'orientation' : [0, 0,0.488591661957, -0.872512571752]},
    'cartonT3' : {'position' : [9.64431185359, 3.82717509192], 'orientation' : [0, 0,0.522462487987, -0.852662271152]},
    'cartonT4' : {'position' : [11.1385987029, 5.34539711938], 'orientation' : [0, 0,0.554163397577, -0.832407910093]},
    'cartonT5' : {'position' : [11.5707310375, 5.83859322984], 'orientation' : [0, 0,-0.249543422567, -0.968363609526]},
    'FOURTH' : {'position' : [-8.72577413744, 1.37004444131], 'orientation' : [0, 0,0.594076067067, -0.804408867764]},
    'cartonF1' : {'position' : [5.38818370378, 1.01750156532], 'orientation' : [0, 0,0.744625679164, -0.667482282859]},
    'cartonF2' : {'position' : [7.33486636806, 1.19437238503], 'orientation' : [0, 0,0.737375806562, -0.675482731014]},
    'cartonF3' : {'position' : [9.33677300848, 1.42572723235], 'orientation' : [0, 0,0.718621021669, -0.695401917754]},
    'cartonF4' : {'position' : [11.5605323907, 1.40929979652], 'orientation' : [0, 0,0.757101970763, -0.653296721151]},
    'cartonF5' : {'position' : [12.1553619393, 1.48928948907], 'orientation' : [0, 0,0.424838891105, -0.905268974728]},
    'FIFTH' : {'position' : [-5.70537084444, 11.9772591602], 'orientation' : [0, 0,0.889437454931, -0.457056904297]},
    'cartonfi_1' : {'position' : [2.45155684982, 0.724971016589], 'orientation' : [0, 0,0.947775468279, -0.318938335308]},
    'cartonfi_2' : {'position' : [4.15352569544, -0.171099875454], 'orientation' : [0, 0,0.895239732518, -0.445584808226]},
    'cartonfi_3' : {'position' : [6.27028416155, -0.88880261294], 'orientation' : [0, 0,0.876234237054, -0.481885423948]},
    'cartonfi_4' : {'position' : [8.05582364884, -1.88320426773], 'orientation' : [0, 0,0.955272163518, -0.295728073756]},
    'cartonfi_5' : {'position' : [8.55217675843, -1.74116515108], 'orientation' : [0, 0,0.238665751518, -0.971101775847]},
}
drop = [-6.85327407238, 14.5195215763, 0, 0, 0.9449442096, -0.32723147884]

message = "next"

def msg_callback(data):
    global message
    message = data.data

def navigation():
    rospy.init_node('navigation', anonymous= True)
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rate = rospy.Rate(10)
    goal = MoveBaseGoal()
    idx = 0
    row = 0
    while(not ac.wait_for_server(rospy.Duration(5))):
        rospy.loginfo('WAITING FOR THE SERVER TO START')

    goal.target_pose.header.frame_id ='map'
    goal.target_pose.header.stamp = rospy.Time.now()

    pub = rospy.Publisher('chatter', String, queue_size=10)
    feedback = String()
    feedback.data = 'next'
    pub.publish(feedback)

    rospy.Subscriber('chatter', String, msg_callback)

    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        try:
            if message == 'next':
                row =+1
                cord = carton[idx]


                goal.target_pose.pose.position.x = coordinates[cord]['position'][0]
                goal.target_pose.pose.position.y = coordinates[cord]['position'][1]
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = coordinates[cord]['orientation'][0]
                goal.target_pose.pose.orientation.y = coordinates[cord]['orientation'][1]
                goal.target_pose.pose.orientation.z = coordinates[cord]['orientation'][2]
                goal.target_pose.pose.orientation.w = coordinates[cord]['orientation'][3]
                print(goal.target_pose)

                # rospy.loginfo(f"SENDING GOAL INFORMATION {coordinates[list(coordinates.keys())[idx]]}")
                ac.send_goal(goal)
                ac.wait_for_result(rospy.Duration(60))
                rate.sleep()

                if (ac.get_state() == GoalStatus.SUCCEEDED):
                    rospy.loginfo("REACHED THE DESTINATION")
                    idx += 1
                else:
                    rospy.loginfo("THE BOT DIDN'T REACH ITS DESTINATION, RETRYING")

                if len(coordinates) == (idx-1):
                    rospy.signal_shutdown()
            elif message == 'next_row':
                idx = cord.index(rows[row])
                cord = carton[idx]

                goal.target_pose.pose.position.x = coordinates[cord]['position'][0]
                goal.target_pose.pose.position.y = coordinates[cord]['position'][1]
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = coordinates[cord]['orientation'][0]
                goal.target_pose.pose.orientation.y = coordinates[cord]['orientation'][1]
                goal.target_pose.pose.orientation.z = coordinates[cord]['orientation'][2]
                goal.target_pose.pose.orientation.w = coordinates[cord]['orientation'][3]
                print(goal.target_pose)

                # rospy.loginfo(f"SENDING GOAL INFORMATION {coordinates[list(coordinates.keys())[idx]]}")
                ac.send_goal(goal)
                ac.wait_for_result(rospy.Duration(60))
                rate.sleep()

                if (ac.get_state() == GoalStatus.SUCCEEDED):
                    rospy.loginfo("REACHED THE DESTINATION")
                    idx += 1
                else:
                    rospy.loginfo("THE BOT DIDN'T REACH ITS DESTINATION, RETRYING")

                if len(coordinates) == (idx-1):
                    rospy.signal_shutdown()

            if message == 'stop':
                speed = Twist()
                speed.angular.w = 0
                speed.linear.x = 0
                vel.publish(speed)
                feedback.data = 'execute'
                pub.publish(feedback)

            if message == 'drop':

                goal.target_pose.pose.position=Point(drop[0],drop[1],0)#contains a point in free space
                goal.target_pose.pose.orientation.x = drop[2]
                goal.target_pose.pose.orientation.y = drop[3]
                goal.target_pose.pose.orientation.z = drop[4]
                goal.target_pose.pose.orientation.w = drop[5]

                # rospy.loginfo(f"SENDING GOAL INFORMATION : DROP POINT")
                ac.send_goal(goal)
                ac.wait_for_result(rospy.Duration(60))
                rate.sleep()

                if (ac.get_state() == GoalStatus.SUCCEEDED):
                    rospy.loginfo("REACHED THE DESTINATION")
                    idx += 1
                    feedback.data = 'DROP'
                    pub.publish(feedback)
                else:
                    rospy.loginfo("THE BOT DIDN'T REACH ITS DESTINATION, RETRYING")
            
        except:
            pass

if __name__ == "__main__":
    navigation()
