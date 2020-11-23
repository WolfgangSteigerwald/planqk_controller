#!/usr/bin/env python
# coding: utf8

import rospy 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg  import Odometry
from math import atan2, pow, sqrt, pi
import ConfigParser 
import json



class TurtleBot:

    def __init__(self):
        
        rospy.init_node('tbt3_1_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/tbt3_1/cmd_vel', 
                                                   Twist, queue_size=1)

        self.pose_subscriber = rospy.Subscriber('/tbt3_1/odom',
                                                Odometry, self.update_pose)
       
        self.pose = Pose()
        self.yaw =0.0
        self.rate = rospy.Rate(20)
        
    def abs_of_twist(self, x):
        if x >= 0:
            return x
        else:
            x = x * -1
            return x

        
    def get_rotation (self, msg):
        roll = pitch = yaw = 0.0
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.yaw = yaw

    def update_pose(self, data):
        self.pose.position = data.pose.pose.position
        self.pose.position.x = round(self.pose.position.x, 4)
        self.pose.position.y = round(self.pose.position.y, 4)
        self.get_rotation(data)
        
    def euclidean_distance(self, goal_pose):
       return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
                    pow((goal_pose.position.y - self.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.0):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        angle = atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)
        return angle

       
    def angular_vel(self, goal_pose, constant=1):
        angle = self.steering_angle(goal_pose)
        print('angle: ' + str(angle))
        print('yaw: ', str(self.yaw))
        print('------')
        return_val = 0.0
        

        if (angle <= 0 ):
            if (self.yaw <= 0):
                print('---1----')
                return_val = angle - self.yaw
            if (self.yaw > 0):
                print('---2----')
                return_val = angle - self.yaw
                if (return_val > pi) or (return_val < -pi):
                    print('---2.1----')
                    if abs(angle) > abs(self.yaw):
                        print('---2.1.1----')
                        return_val = -1 * (angle + self.yaw)
                    else:
                        print('---2.1.2----')
                        return_val = angle + self.yaw

        if (angle > 0 ):
            if (self.yaw < 0):
                print('---3----')
                return_val = angle - self.yaw
                if (return_val > pi) or (return_val < -pi):
                    print('---3.1----')    
                    if abs(angle) > abs(self.yaw):
                        print('---3.1.1----')
                        return_val = -1 * (angle - self.yaw)
                    else:
                        print('---3.1.2----')
                        return_val = -1* angle + self.yaw

            if (self.yaw >= 0):
                print('---4----')
                return_val = angle - self.yaw

        if ((angle > 3.41) or (angle < 3,41)) and (self.yaw == 0):
           return_val = 0,1
                
        print('return_val :' + str(return_val))
        if (return_val > 0.8):
            return_val = 0.8
        if (return_val < -0.8):
            return_val = -0.8
        
        print('return_val nach :' + str(return_val))
        print('distanz nach :' + str(self.euclidean_distance(goal_pose)))
        print('------------------------')
        
        return return_val

    def set_intial_direction(self, goal_pose):
        
        vel_z = self.angular_vel(goal_pose)
        vel_msg = Twist()
        vel_msg = initializeTwist(vel_msg)
            #print('vel_z 1: ' + str(vel_z))

        while self.abs_of_twist(vel_z) > 0.1: 
            vel_z =1.0 * self.angular_vel(goal_pose)
 
            #print('vel_z : ' + str(vel_z))
            vel_msg.angular.z = vel_z

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()
        


    def move2goal(self, step):
        goal_pose = Pose()
        goal_pose.position.x = step.position.x
        if step.position.y > 0:
             goal_pose.position.y = step.position.y 
        else:
             goal_pose.position.y = step.position.y
            
        print('goal_pose.position.x ' + str(goal_pose.position.x))
        print('goal_pose.position.y ' + str(goal_pose.position.y))
        distance_tolerance = float(0.02)

        vel_msg = Twist()
        vel_msg = initializeTwist(vel_msg)
        
        print('beginn while')
        i = 0
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            #print(self.euclidean_distance(goal_pose))
            if i == 0:
                print('set_intial_direction-beginn')
                self.set_intial_direction(goal_pose)
                i = i+1
                print('set_intial_direction-end')
            # Linear velocity in the x-axis.
            vel_x = self.linear_vel(goal_pose)
#            print('vel_x pre : ' + str(vel_x))
            if vel_x > 0.3:
                vel_x = 0.3
            #print('vel_x post : ' + str(vel_x))
            vel_msg.linear.x = vel_x
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_z =  self.angular_vel(goal_pose)
            #print('vel_z nach : ' + str(vel_z))
            vel_msg.angular.z = vel_z
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
        
        print('endwhile')
        
        vel_msg = initializeTwist(vel_msg)
            
        self.velocity_publisher.publish(vel_msg)
        
        return True

def initializeTwist(twist_msg):
    twist_msg.linear.x = 0.000
    twist_msg.linear.y = 0.000
    twist_msg.linear.z = 0.000
    twist_msg.angular.x = 0.000
    twist_msg.angular.y = 0.000
    twist_msg.angular.z = 0.000
    return twist_msg


def getObjectByType(config, mtype):
    objectcount = config.getint('containing', 'object-count')
    print('getObjectByType')
    print(objectcount)
    models = []
    j=0
    for i in range(1, objectcount+1):
        object = 'object-' + str(i)
        model = dict(config.items(object))
        if model['type'] == mtype:
            models.append(model)
            j = j +1
    print(mtype, j)
    return models
    
def getRobotPath(filename):
    with open(filename, 'r') as myfile:
        data=myfile.read()

    # parse file
    obj = json.loads(data)
    #print("positions: " + str(obj['positions']))
    path = obj['transport']
    return path

def getCoordinates(objects, i, where):
    step = Pose()
    if  where == 'start':
        first = i[0]
    elif  where == 'target':
        first = i[1]
    
    second = objects[first]
    step.position.x = float(second['pos-x'])
    step.position.y = float(second['pos-y'])
    if second['type'] == 'machine':
        if step.position.y > 0.0:
            step.position.y = step.position.y - 0.20
        else: 
            step.position.y = step.position.y + 0.20
        
    #print('Coo step')
    #print(i)
    return step
            
   
if __name__=="__main__":
    robots = []
    controller = {}
    #print('pre config')
    config = ConfigParser.ConfigParser()
    config.readfp(open('../conf/insert-obj.ini'))
    robots = getObjectByType(config, 'robot') 
    try:
        #rospy.init_node ("bot_controller")
#!!WS ----hier muessen die Objewkte für jeden Bot erzeugt werden
        tbt3 = TurtleBot()
        #print(robots[0])
        my_object = robots[0]
#        tbt3.pose.position.x = float(my_object['pos-x'])
#        tbt3.pose.position.y = float(my_object['pos-y'])
#        tbt3.pose.position.z = float(my_object['pos-z'])
        machines = getObjectByType(config,'machine')
        starts = getObjectByType(config, 'startpoint')
        targets = getObjectByType(config, 'targetpoint')
        my_points = starts + machines + targets
        path = getRobotPath('../conf/jso4x4solution.json')
        #print(path)
        j = 0
        z = len(path)
        #print('z : ' + str(z))
        for i in path :
            print(i)
            print('step: ' + str(j))
            step = getCoordinates(my_points, i, 'start')
            result = tbt3.move2goal(step)
            step = getCoordinates(my_points, i, 'target')
            result = tbt3.move2goal(step)
            j = j + 1
            
        print(j)
        #if result:
        rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        
 