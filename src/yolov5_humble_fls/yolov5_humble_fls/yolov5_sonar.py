#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import sys
#conda_base_addr = '/home/pcl-02/anaconda3'    # your conda address
#conda_env = 'py310'                          # your conda environment
#python_env = 'python3.10'                      # python environment in your conda environment 
#sys.path.append(conda_base_addr + '/envs/' + conda_env + '/lib/' + python_env + '/site-packages/')

import cv2
import torch
import rclpy
from rclpy.lifecycle import Node
import numpy as np
from sensor_msgs.msg import Image
from object_msgs.msg import ObjectAzimuthRange
from object_msgs.msg import ObjectsAzimuthRange

import sys

class Yolo_Dect(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        self.declare_parameter('yolov5_path', '')
        yolov5_path = self.get_parameter('yolov5_path').get_parameter_value().string_value
        self.declare_parameter('weight_path', '')
        weight_path = self.get_parameter('weight_path').get_parameter_value().string_value
        self.declare_parameter('image_topic','/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.declare_parameter('objects_info_pub_topic','/objects_info')
        objects_info_pub_topic = self.get_parameter('objects_info_pub_topic').get_parameter_value().string_value
        self.declare_parameter('conf',0.5)
        conf = self.get_parameter('conf').value
        self.declare_parameter('display_image', False)
        self.display_image = self.get_parameter('display_image').get_parameter_value().bool_value

        # load local repository(YoloV5:v7.0)
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')       ######
        # which device will be used
        self.declare_parameter('use_cpu', False)
        use_cpu = self.get_parameter('use_cpu').get_parameter_value().bool_value
        if use_cpu:
            self.model.cpu()    ######                                                                       
            self.get_logger().info("use_cpu")
        else:
            self.model.cuda()   ######
            self.get_logger().info("use_cuda")
        #self.model.conf = float(conf)  ######
        self.model.conf = conf  ######
        self.color_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 1)
        # output publishers
        self.objects_info_pub = self.create_publisher(ObjectsAzimuthRange, objects_info_pub_topic, 1)

        timer = Node.create_timer(self, 2, self.timer_callback)
            
    def timer_callback(self):
        if(not self.getImageStatus):
            self.get_logger().info("waiting for image.")

    def image_callback(self, image):
        self.getImageStatus = True
        self.objects_azimuth_range = ObjectsAzimuthRange()
        self.objects_azimuth_range.header = image.header
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image)                   ######
        # xmin    ymin    xmax    ymax    confidence    class    name

        boxs = results.pandas().xyxy[0].values

        # loading the range and azimuth of sonar
        sonar_info = image.header.frame_id
        mid_index = sonar_info.find(' ')
        sonar_azimuth = float(sonar_info[1 : mid_index])
        sonar_range = float(sonar_info[mid_index + 1 : -1])

        self.dectshow(self.color_image, boxs, sonar_azimuth, sonar_range)
        cv2.waitKey(3)

    def dectshow(self, org_img, boxs, sonar_azimuth, sonar_range):
        img = org_img.copy()

        count = 0
        for i in boxs:
            count += 1

        image_height = img.shape[0]
        image_width = img.shape[1]
        
        for box in boxs:
            # calculate the azimuth and range info and package the info for ros
            object_azimuth_range = ObjectAzimuthRange()
            object_azimuth_range.class_name = box[-1]
            object_azimuth_range.probability = np.float64(box[4])
            object_x = (int(box[0]) + int(box[2])) / 2.0                          #width
            object_y = (int(box[1]) + int(box[3])) / 2.0                          #height
            object_azimuth_range.object_azimuth = (object_x / (image_width / 2.0) - 1.0) * (sonar_azimuth / 2.0)
            object_azimuth_range.object_range = (1.0 - object_y / image_height) * (sonar_range)
            object_azimuth_range.xmin = int(box[0])
            object_azimuth_range.ymin = int(box[1])
            object_azimuth_range.xmax = int(box[2])
            object_azimuth_range.ymax = int(box[3])
            object_azimuth_range.num = int(count)
            self.objects_azimuth_range.object_azimuth_range.append(object_azimuth_range)
            
            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0,183,3)
                self.classes_colors[box[-1]] = color

            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (int(color[0]), int(color[1]), int(color[2])), 2)

            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10

            object_pro_str = object_azimuth_str = '%.2f' % object_azimuth_range.probability
            object_azimuth_str = '%.2f' % object_azimuth_range.object_azimuth
            object_range_str = '%.2f' % object_azimuth_range.object_range
            object_pose_str = '(' + object_azimuth_str + ', ' + object_range_str + ')'
            text = box[-1] + '-' + object_pro_str + ': ' + object_pose_str
            cv2.putText(img, text, (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        self.objects_info_pub.publish(self.objects_azimuth_range)
        
        if self.display_image:
            cv2.imshow('YOLOv5', img)

def main(args=None):
    rclpy.init(args=args)
    yolo_dect_node = Yolo_Dect()
    rclpy.spin(yolo_dect_node)
    yolo_dect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
