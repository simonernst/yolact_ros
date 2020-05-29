#!/usr/bin/env python3

import os, sys
import numpy
import math
import statistics

import rospy


from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from yolact_ros_msgs.msg import Detections
from yolact_ros_msgs.msg import Detection
from yolact_ros_msgs.msg import Box
from yolact_ros_msgs.msg import Mask


#Détection pendant robot en mouvement ?




class MaskTo3D(object):
    
    nok_class = ['person','car','bycicle','motorcycle','airplane','bus','train','truck','boat','traffic light','fire hydrant'
                'stop sign','parking meter','sheep','cow','elephant','bear','zebra','giraffe','sheep','horse']
    ok_class = []
    
    #change to ros.get_param
    depth_topic="/camera/depth/points"



    def __init__(self):
        rospy.init_node("depth_mask",anonymous=True)

        self.sub_mask_msg=rospy.Subscriber(mask_msg,Detections,self.mask_msg_callback)
        sekf.cloud_msg=PointCloud2()

    def mask_msg_callback(self,req):
        array=rosnp.point_cloud2.pointcloud2_to_xyz_array(self.cloud_msg,False)
        label,score,arrayX,arrayY =[],[],[],[]


        for item in req.detections:
            if item.class_name in nok_class:
                #object not interesting
                pass

            else:
                label.append()






if __name__=="__main__":
    a=MaskTo3D()
    while not rospy.is_shutdown():
        a.rate.sleep()

