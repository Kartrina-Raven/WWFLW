#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from ultralytics import YOLO

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov11_ros_msgs.msg import BoundingBox, BoundingBoxes


class Yolo_Dect:
    def __init__(self):
        # 加载参数
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov11/BoundingBoxes')
        mask_topic = rospy.get_param('~mask_topic', '/yolov11/mask_image')
        mask_viz_topic = rospy.get_param('~mask_viz_topic', '/yolov11/mask_image_viz')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        self.conf = float(rospy.get_param('~conf', 0.8))
        self.visualize = rospy.get_param('~visualize', True)
        self.task = rospy.get_param('~task', 'segment')
        self.retina_masks = rospy.get_param('~retina_masks', True)

        # 设置推理使用的设备
        if rospy.get_param('/use_gpu', False):
            self.device = 'cuda'
        else:
            self.device = 'cpu'

        self.model = YOLO(weight_path, task=self.task)
        self.model.fuse()
        self.color_image = Image()
        self.getImageStatus = False

        # 加载类别颜色
        self.classes_colors = {}

        # 订阅图像话题
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # 输出发布器
        self.position_pub = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher(
            '/yolov11/detection_image',  Image, queue_size=1)

        self.mask_pub = rospy.Publisher(
            mask_topic, Image, queue_size=1)

        # 等待图像消息
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        results = self.model.predict(
            source=self.color_image,
            conf=self.conf,
            device=self.device,
            retina_masks=self.retina_masks,
            verbose=False
        )

        self.dectshow(results, image.header, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, results, image_header, height, width):
        result = results[0]
        has_masks = result.masks is not None and len(result.masks) > 0
        self.frame = result.plot(boxes=True, masks=True)
        mask_image = self.build_mask_image(result, height, width) * 255

        inference_time = result.speed.get('inference', 0.0)
        if inference_time > 0:
            fps = 1000.0 / inference_time
            cv2.putText(self.frame, f'FPS: {int(fps)}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2, cv2.LINE_AA)

        for box in result.boxes:
            boundingBox = BoundingBox()
            boundingBox.xmin = np.int64(box.xyxy[0][0].item())
            boundingBox.ymin = np.int64(box.xyxy[0][1].item())
            boundingBox.xmax = np.int64(box.xyxy[0][2].item())
            boundingBox.ymax = np.int64(box.xyxy[0][3].item())
            boundingBox.Class = result.names[int(box.cls.item())]
            boundingBox.probability = box.conf.item()
            self.boundingBoxes.bounding_boxes.append(boundingBox)

        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(self.frame, image_header, 'bgr8', width * 3, self.image_pub)
        self.publish_image(mask_image, image_header, 'mono8', width, self.mask_pub)

        if self.visualize:
            window_name = 'YOLOv11 Segmentation' if has_masks else 'YOLOv11 Detection'
            new_width = width // 2
            new_height = height // 2
            resized_frame_for_display = cv2.resize(self.frame, (new_width, new_height))
            cv2.imshow(window_name, resized_frame_for_display)

            if self.task == 'segment' and not has_masks:
                rospy.logwarn_throttle(
                    5.0,
                    'Model returned no masks. Make sure ~weight_path points to a segmentation model.'
                )

    def build_mask_image(self, result, height, width):
        mask_image = np.zeros((height, width), dtype=np.uint8)
        if result.masks is None or len(result.masks) == 0:
            return mask_image

        masks = result.masks.data
        if hasattr(masks, 'cpu'):
            masks = masks.cpu().numpy()

        # 将所有实例掩码合并为一张二值占用图像。
        mask_image = np.any(masks > 0.5, axis=0).astype(np.uint8)
        return mask_image

    def publish_image(self, imgdata, image_header, encoding, step, publisher):
        image_temp = Image()
        header = Header()
        header.stamp = image_header.stamp if image_header.stamp else rospy.Time.now()
        header.frame_id = self.camera_frame if self.camera_frame else image_header.frame_id
        image_temp.height = imgdata.shape[0]
        image_temp.width = imgdata.shape[1]
        image_temp.encoding = encoding
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = step
        publisher.publish(image_temp)


def main():
    rospy.init_node('yolov11_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":
    main()