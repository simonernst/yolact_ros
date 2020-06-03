#!/usr/bin/env python3

import os, sys
import numpy as np
import math
import statistics
import matplotlib.pyplot as plt
from matplotlib.path import Path

import rospy
import ros_numpy as rosnp

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from yolact_ros_msgs.msg import Detections
from yolact_ros_msgs.msg import Detection
from yolact_ros_msgs.msg import Box
from yolact_ros_msgs.msg import Mask
from yolact_ros_msgs.msg import Mask_Depth, Mask_Coordinates

import skimage.data
#Debug printing whole array
np.set_printoptions(threshold=sys.maxsize)


#USE CASE a prendre en compte
#Détection pendant robot en mouvement ?




class MaskTo3D(object):
    
    nok_class = ['person','car','bycicle','motorcycle','airplane','bus','train','truck','boat','traffic light','fire hydrant'
                'stop sign','parking meter','sheep','cow','elephant','bear','zebra','giraffe','sheep','horse']
    ok_class = []

    #change to ros.get_param
    depth_topic="/camera/depth/points"



    def __init__(self):
        rospy.init_node("depth_mask",anonymous=True)
        mask_msg="/yolact_ros/detections"
        self.pointcloud_sub=rospy.Subscriber("/camera/depth/points", PointCloud2, self.pcl_callback)
        self.sub_mask_msg=rospy.Subscriber(mask_msg,Detections,self.mask_msg_callback)
        self.pub_mask_depth=rospy.Publisher("/mask_coordinates", Mask_Depth, queue_size=1)
        self.cloud_msg=PointCloud2()
        self.rate=rospy.Rate(10)
        #fig,self.ax=plt.subplots()
        #plt.show(block=True)


    def pcl_callback(self,req):
        #get the data of the PointCloud from RGBD camera
        self.cloud_msg=req

    def mask_msg_callback(self,req):
        img=skimage.data.chelsea()
        array=rosnp.point_cloud2.pointcloud2_to_xyz_array(self.cloud_msg,False)
        label_store,score_store,box_store,mask_store,path_points =[],[],[],[],[]

        #custom msg creation with 3D coordinates of each pixel in each masks
        message=Mask_Depth()
        message.header.stamp=rospy.Time.now()

        rospy.loginfo("Parsing detections")
        for item in req.detections:
            label_store.append(item.class_name)
            score_store.append(item.score)
            box_store.append(item.box)
            mask_store.append(item.mask)

        for i in range(len(label_store)):
            #print(box_store[i].x1,box_store[i].x2, box_store[i].y1,box_store[i].y2)
            #Think of retrieving img size from Image topic
            X,Y = np.mgrid[:img.shape[1], :img.shape[0]]
            points = np.vstack((X.ravel(),Y.ravel())).T
            vertices = np.asarray([(box_store[i].x1,box_store[i].y1),
                                   (box_store[i].x1,box_store[i].y2),
                                   (box_store[i].x2,box_store[i].y1),
                                   (box_store[i].x2,box_store[i].y2)])
            path = Path(vertices)        
            mask=path.contains_points(points)
            path_points.append(points[np.where(mask)])
            #print(type(path_points))
        #print(path_points[0][0][0])
        #print(path_points[0][0][1])
        #print(len(label_store),len(score_store),len(box_store),len(mask_store),len(path_points))


        for i in range(len(label_store)):
             #create custom message with 3D points of mask
            mask_message = Mask_Coordinates()
            mask_message.class_name = label_store[i]
            mask_message.score = score_store[i]

            list_x,list_y,list_z = [],[],[]
            score=score_store[i]
            for j in range(0,len(path_points[i])):
                #print(array[path_points[i][j][0]][path_points[i][j][1]][0])
                #print(path_points[i][j][0])
                #print(path_points[i][j][1])
                if math.isnan(array[path_points[i][j][0]][path_points[i][j][1]][0]) or \
                   math.isnan(array[path_points[i][j][0]][path_points[i][j][1]][1]) or \
                   math.isnan(array[path_points[i][j][0]][path_points[i][j][1]][2]):
                    pass
                else:
                    list_x.append(array[path_points[i][j][0]][path_points[i][j][1]][0])
                    list_y.append(array[path_points[i][j][0]][path_points[i][j][1]][1])
                    list_z.append(array[path_points[i][j][0]][path_points[i][j][1]][2])
            
            mask_message.xcoord = list_x
            mask_message.ycoord = list_y
            mask_message.zcoord = list_z
            message.coordinates.append(mask_message)
        self.pub_mask_depth.publish(message)

if __name__=="__main__":
    a=MaskTo3D()
    while not rospy.is_shutdown():
        a.rate.sleep()

