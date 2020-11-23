#!/usr/bin/env python

import time
import rospy
import tf 
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg  import Odometry
from tf.transformations import euler_from_quaternion
import sys
import roslib
import os
import ConfigParser 
from getkey import getkey, keys

config = ConfigParser.ConfigParser()
config.readfp(open('../conf/insert-obj.ini'))








def modelspawn(name, fname, pose, ftype):
    #print(name + ',' + fname + ',' + 'pose' + ',' + modelformat)
    #f = open(fname,'r')
    #model_xml = f.read()
    print(fname)
    p = os.popen("rosrun xacro xacro.py " + fname)
#    p = os.popen("rosrun xacro --inorder" + fname)
    xml_string = p.read()
    p.close()
    
    if ftype == "sdf":
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
    if ftype == "urdf":
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        if ftype == "sdf":
            spawnsrv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        if ftype == "urdf":
            spawnsrv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        
        #test = spawnsrv(name, model_xml, name, pose, None)
        test = spawnsrv(name, xml_string, name, pose, None)
        print("test")
        print(test)
        
    except rospy.ServiceException:
        print ("Service call failed %s" % sys.exc_info()[0])

def modeldelete(name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        deletesrv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        deletesrv(name)
    except rospy.ServiceException:
        print ("Service call failed %s" % e)

def getmodel(config, num):
    object = "object-"
    object = object + str(num)
    #print("object " + object)
    model = config.items(object)
    return model

def setposition(object,pose):
    if object['position'] == 'True':
        pose.position.x = float(object['pos-x'])
        pose.position.y = float(object['pos-y'])
        pose.position.z = float(object['pos-z'])

    if object['orientation'] == 'True':
        pose.orientation.x = float(object['orient-x'])
        pose.orientation.y = float(object['orient-x'])
        pose.orientation.z = float(object['orient-x'])
        pose.orientation.x = float(object['orient-x'])
    #print('position =')
    #print(pose)
    return pose

def initialise_position(object):
    model = object['name']
    state_msg = ModelState()
    state_msg.model_name = model
    state_msg.pose.position.x = float(object['pos-x'])
    state_msg.pose.position.y = float(object['pos-y'])
    state_msg.pose.position.z = float(object['pos-z'])
    if object['orientation'] == 'True':
        state_msg.pose.orientation.x = float(object['orient-x'])
        state_msg.pose.orientation.y = float(object['orient-x'])
        state_msg.pose.orientation.z = float(object['orient-x'])
        state_msg.pose.orientation.w = float(object['orient-x'])
    else:
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        state_msg.pose.orientation.w = 0.0

    state_msg.twist.linear.x = 0.0000
    state_msg.twist.linear.y = 0.0000
    state_msg.twist.linear.z = 0.0000
    state_msg.twist.angular.x = 0.0000
    state_msg.twist.angular.y = 0.0000
    state_msg.twist.angular.z  = 0.0000
    #print('state_msg')
    #print(state_msg)
    vel_msg = Twist()
    pub_name = '/'+ model + '/cmd_vel'
    velocity_publisher = rospy.Publisher(pub_name, Twist, queue_size=1)
#    velocity_publisher = rospy.Publisher('/tbt3_1/cmd_vel', 
#                                                   Twist, queue_size=1)
    vel_msg.linear.x = 0.000
    vel_msg.linear.y = 0.000
    vel_msg.linear.z = 0.000
    vel_msg.angular.x = 0.000
    vel_msg.angular.y = 0.000
    vel_msg.angular.z = 0.000
            
    velocity_publisher.publish(vel_msg)

    
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        print('resp:')
        print(resp)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = object['name']
    resp_coordinates = model_coordinates(model, model)
    print('resp_coordinates')
    print(resp_coordinates)
    #pub_name = name + '/cmd_vel'
    my_pub = rospy.Publisher('/gazebo/model_state', ModelState, queue_size=10)
    my_pub.publish(state_msg)

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
    
def distribute_machine(machines, dist_area):
    count = len(machines)
    dist_x = dist_area['dist-x']
    line2 = count / 2
    line1 = count - line2
    
        
        
        


if __name__ == '__main__':
    
    objectcount = config.getint('containing', 'object-count')
    machincount = config.getint('containing', 'machine-count')
    partcount = config.getint('containing', 'part-count')
    startcount = config.getint('containing', 'start-count')
    targetcount = config.getint('containing', 'target-count')
    modelpath = config.get('containing', 'model-path')
    distarea = config.getboolean('containing', 'dist-area')
    my_machines = getObjectByType(config, 'machine')
    dist_area = getObjectByType(config, 'dist_area')
    #distribute_machine(my_machines, dist_area)
    path = ' '
    rospy.init_node ("insert_obj")
    for i in range(1, objectcount+1):
        object = getmodel(config, i)
        myobjdict = dict(object)
        
        modelformat = myobjdict['format']
        if myobjdict['active'] == 'True' and modelformat != 'non':
            name = myobjdict['name']
            #print('name>=' + name)
            #print('format=' + modelformat)
            if modelformat == 'urdf':
                path = '/home/wolf/catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro'
            elif modelformat == 'sdf':
                path = modelpath + name + '/model.sdf'
            pose = Pose()
            
            pose = setposition(myobjdict, pose)
            #print(pose)
            #print(path)
            print ("spawn model: " + name)
            
            modelspawn(name, path, pose, modelformat)
                    
            if myobjdict['type'] == 'robot':
                initialise_position(myobjdict)
            
        #time.sleep(1)
    #tuto = Tutorial()
    #tuto.show_gazebo_models()
    end = False
    
    while end  != True:
        print('press q to quit')
        print('press i to reset turtlebot')
        print('press g to set a goal')
        key = getkey()
        if key == 'q':
            end = True
            break            
        elif key == 'i':
            for i in range(1, objectcount+1):
                object = getmodel(config, i)
                myobjdict = dict(object)
                if myobjdict['type'] == 'robot':
                    initialise_position(myobjdict)
        elif key == 'g':
            for i in range(1, objectcount+1):
                object = getmodel(config, i)
                myobjdict = dict(object)
                if myobjdict['type'] == 'robot':
                    send_to_goal(myobjdict)
        #rospy.spin()
        

    #time.sleep(10)
#    for i in range(1, objectcount+1):
    for i in range(objectcount, 0, -1):
        object = getmodel(config, i)
        myobjdict = dict(object)
        name = myobjdict['name']
        print ("delete model: " + name)
        modeldelete(name)
        
    
    
    
    
    
    
    
    