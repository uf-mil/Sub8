#!/usr/bin/python
#TODO
#Get modularity working
#Generate Services using a for loop
#Step 1: Parse Launch File into this so I can read new models and coords directly from there
#Step 2: Figure out which service is accessing master pose function
#Step 3: Making master junk functions
#Step 4: Make the whole thing a class <<< This should come first.
import cv2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Vector3Stamped, Vector3
from mil_misc_tools import text_effects
from mil_msgs.srv import SetGeometry
import numpy as np
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest, VisionRequest2D, VisionRequest2DResponse
from std_msgs.msg import Header, Float64
import os
import rospkg

import sys
import time
from tf import transformations
import yaml
import xml.etree.ElementTree
rospack = rospkg.RosPack()
#Find a way to parse through the yaml correctly. Might be unneccessary but nice.
config_file = os.path.join(rospack.get_path('sub8_missions'), 'sub8', 'vision_proxies.yaml')
f = yaml.load(open(config_file, 'r'))
launch_file = os.path.join(rospack.get_path('sub8_gazebo'), 'launch', 'duck.launch')
launch = xml.etree.ElementTree.parse(launch_file).getroot()

fprint = text_effects.FprintFactory(title="SIMULATOR").fprint

def handle_fake_perception(extra,resp):
    now = rospy.get_rostime()
    k = np.uint32(0)
    if extra != '':
        resp = extra
    if resp == '':
        print "NO TARGET"
        sys.exit(0)
    for node in launch.iter('node'):
        name = node.get('name')
        if name == resp:
            v = node.get('args')
            point = getCoords(resp,v)
            q = getOrient(resp,v)

    pose_stamp = PoseStamped(header = Header(seq = k, stamp = now, frame_id = "/map"),
        pose = Pose(position=point,
        orientation=Quaternion(*q)))
    covariance_diagonal = Vector3(0,0,0)
    found = True
    resp2 = VisionRequestResponse(pose_stamp,covariance_diagonal,found)
    return resp2
def getOrient(name,v):
    #Find out X orient
    j = v.find('-X')
    start = j+3
    end = v.find(' ', start)
    XX = (float)(v[start:end])

    #Find our Y orient
    j = v.find('-Y')
    start = j+3
    end = v.find(' ', start)
    YY = (float)(v[start:end])

    #Find our Z orient
    j = v.find('-Z')
    start = j+3
    end = v.find(' ', start)
    ZZ = (float)(v[start:end])

    response = transformations.quaternion_from_euler(XX,ZZ,YY)
    return response
def getCoords(name,v):
    #Im going to need to edit our launch file to match all the names of the objects in the launch file to the names of the target objects in this
    #On top of that I need to centralize all pose and orientation numbers into the launch file, moving them out of the STL files.
    #After that this should be relatively easy to implement, but time consuming.
    #IF target_name != ''
        #run get coords while target_name != to previous target_name
    #Find our x.
    j = v.find('-x')
    start = j+3
    end = v.find(' ', start)
    x = (float)(v[start:end])

    #Find our y
    j = v.find('-y')
    start = j+3
    end = v.find(' ', start)
    y = (float)(v[start:end])

    #Find our z
    j = v.find('-z')
    start = j+3
    end = v.find(' ', start)
    z = (float)(v[start:end])
    return Point((x-13.0),(y-24.0),(-1))
def set_geometry(req):
    return {'success': True}
def vision_cb_2D():
    return True
def start(resp):
    return SetBoolResponse(True,"")
def init_service(name,target):
    #Generates the important service
    serv = rospy.Service('/vision/'+name+'/pose',VisionRequest, lambda h: handle_fake_perception(h.target_name,target))
    s1 = rospy.Service('/vision/'+name+'/set_geometry', SetGeometry, set_geometry)
    s2 = rospy.Service('/vision/'+name+'/2D', VisionRequest2D, vision_cb_2D)
    s3 = rospy.Service('/vision/'+name+'/enable', SetBool, start)
def fake_perception_server():
    rospy.init_node('fake_perception')
    #Please put name of service and target of service you wish to mimic
    #If the service you are requesting provides a target_name, use '' to indicate that a target is not necessary in the dictionary. See buoys for example.
    d = {'orange_rectangle':'orange_rectangle', 'buoys':''}
    for key in d:
        init_service(key,d[key])

    fprint("Faking perception.")
    rospy.spin()
if __name__ == "__main__":
    fake_perception_server()
