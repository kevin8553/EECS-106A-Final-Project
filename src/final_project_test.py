#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import numpy as np
import tf2_ros
import sys
import numpy as np
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from math import pow, atan2, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def euclidean_distance(self_pose, goal_pose):
        return sqrt(pow((goal_pose.x - self_pose.x), 2) +
                    pow((goal_pose.y - self_pose.y), 2))
                    
def linear_vel(self_pose, goal_pose, constant=0.3):
    return constant * euclidean_distance(self_pose, goal_pose)

def steering_angle(self_pose, goal_pose):
    return atan2(goal_pose.y - self_pose.y, goal_pose.x - self_pose.x) #rel distance between two turtles

def angular_vel(self_pose, goal_pose, constant=10):
    return constant * (steering_angle(self_pose, goal_pose) - self_pose.theta) #Required angle to turn


# def prims_algo(init, goals, end):
#         weight = []
        
#         goals.insert(0, init)
#         goals.append(end)
#         for i in range(len(goals)):
#             weight1 = []
#             for j in range(len(goals)):
#                 weight1.append(round(sqrt(pow((goals[j][0] - goals[i][0]), 2) + pow((goals[j][1] - goals[i][1]), 2)),4))
#             weight.append(weight1)
#         print(weight)

#         INF = 9999999
#         # number of vertices in graph
#         N = 4
#         #creating graph by adjacency matrix method
#         G = weight

#         selected_node = [0, 0, 0, 0]

#         no_edge = 0

#         selected_node[0] = True

#         # printing for edge and weight
#         print("Edge : Weight\n")
#         route = []
#         while (no_edge < N - 1):
        
#             minimum = INF
#             a = 0
#             b = 0
#             for m in range(N):
#                 if selected_node[m]:
#                     for n in range(N):
#                         if ((not selected_node[n]) and G[m][n]):  
#                             # not in selected and there is an edge
#                             if minimum > G[m][n]:
#                                 minimum = G[m][n]
#                                 a = m
#                                 b = n
#             route.append(b)
#             # print(str(a) + "-" + str(b) + ":" + str(G[a][b]))
#             selected_node[b] = True
#             no_edge += 1
#         return route

def get_target_coord(base, target):
  tfBuffer_2 = tf2_ros.Buffer()
  tfListener_2 = tf2_ros.TransformListener(tfBuffer_2)
  ar_tag = ['ar_marker_13', 'ar_marker_16', 'ar_marker_9']
  goals = []
  #print(ar_tag)
  for i in range(len(target)):
    try: 
      trans_target = tfBuffer_2.lookup_transform(base, target[i], rospy.Time(), rospy.Duration(0.5))
      # print(trans_target.transform.translation)
      goal = [round(-trans_target.transform.translation.x/3.704,2), round(-trans_target.transform.translation.y/3.704,2)]
      goals.append(goal)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print(e)
      pass
  return goals


#Define the method which contains the main functionality of the node.
def controller(goalx, goaly):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber("/tf", TFMessage, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  trans = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time())
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  goal = Pose()
  self_pose = Pose()
  goal.x = goalx
  goal.y = goaly

  self_pose.x = 0
  self_pose.y = 0
  self_pose.x = trans.transform.translation.x
  self_pose.y = trans.transform.translation.y
  self_pose.theta = 0
  control_command = Twist() # Generate this

  # Loop until the node is killed with Ctrl-C
  while euclidean_distance(self_pose, goal) >= 0.05:
    try:


      # trans = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time())
      trans = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time())
      # ar_tag = ['ar_marker_13', 'ar_marker_16']
      # for i in range(len(ar_tag)):
      #   trans_test = tfBuffer.lookup_transform("ar_marker_7", ar_tag[i], rospy.Time())
      #   print(i)
      #   print(trans_test.transform.translation)

      
      goal = Pose()
      self_pose = Pose()

      # set the goal position
      goal.x = goalx
      goal.y = goaly

      # update self position (relative to "odom")
      self_pose.x = trans.transform.translation.x
      self_pose.y = trans.transform.translation.y

      # get the orientation of the turtlebot from quaternion
      orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
      [raw, pitch, yaw] = euler_from_quaternion(orientation_list)
      self_pose.theta = yaw

      # setup the twist message
      control_command.linear.x = linear_vel(self_pose, goal)
      control_command.linear.y = 0 
      control_command.linear.z = 0 

      control_command.angular.x = 0
      control_command.angular.y = 0
      control_command.angular.z = angular_vel(self_pose, goal)

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print(e)
      pass
    # Use our rate object to sleep until it is time to publish again
      r.sleep()

    # command turtlebot to stop once reach the target 
    control_command.linear.x = 0
    control_command.angular.z = 0
    pub.publish(control_command)

# Generate random coordinate
def rand_point(coord_range, num):
  goals = []
  for i in range(num):
    coord = []
    coord.append(np.random.randint(0, coord_range))
    coord.append(np.random.randint(0, coord_range))
    goals.append(coord)
  return goals

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)
  goalx = 0
  goaly = 0
  end_x = 0
  end_y = 1
  # coord_range = 2
  # num_of_target = 3
  # base = 'ar_marker_7'
  # target = ['ar_marker_13', 'ar_marker_16', 'ar_marker_9']
  # goals = get_target_coord(base, target)
  # print(goals)

  while not rospy.is_shutdown():
    try:
      # # controller(goalx, goaly)
      # # goals = rand_point(coord_range, num_of_target)
      # print(goals)

      # print("Place the targets")
      
      # # goals = [[0.4,0.4], [0.4, 0.35], [0.3,0.2]]
      # check = input("Start the robot?")
      # route = prims_algo([0,0], goals, [end_x, end_y])
      # print(route)
      

      # for i in range(len(route)):

      #   print(goals[route[i]])
      #   controller(goals[route[i]][0], goals[route[i]][1])
      print("finished")
      print("Go back to origin")
      controller(end_x, end_y)
      print("back to origin!")
    except rospy.ROSInterruptException:
      pass