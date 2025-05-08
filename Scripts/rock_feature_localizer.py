

#!/usr/bin/env python3
import os, pickle, math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge

class RockLocalizerBackproj(Node):
    def __init__(self):
        super().__init__('rock_localizer_backproj')

        # ——— load or create DB ———
        self.db_path = os.path.expanduser('~/rock_features.db')
        if os.path.isfile(self.db_path):
            with open(self.db_path, 'rb') as f:
                self.db = pickle.load(f)
        else:
            self.db = []
        self.get_logger().info(f"DB loaded: {len(self.db)} rocks")

        # ——— GUI odometry subscriber ———
        self.have_odom = False
        self.rx = self.ry = self.rz = 0.0
        self.ryaw = 0.0
        self.create_subscription(Odometry, '/odom_est', self.odom_cb, 10)

        # ——— CV setup ———
        self.bridge = CvBridge()
        self.orb    = cv2.ORB_create(nfeatures=500)

        # ——— HSV thresholds (your “perfect” values) ———
        self.hsv_lower = np.array((  0,   1,   0))
        self.hsv_upper = np.array((117, 150,  78))

        # ——— **more lenient** filtering params ———
        self.min_area     =  20    # was 50
        self.max_area     = 20000
        self.min_solidity =   0.1  # was 0.2
        self.min_features =   0    # was 3
        self.new_thresh   = 0.2

        # ——— publishers ———
        self.mask_pub = self.create_publisher(Image,       'rock_mask',    1)
        self.img_pub  = self.create_publisher(Image,       'detections',   1)
        self.mkr_pub  = self.create_publisher(MarkerArray, 'rock_markers', 1)

        # ——— sync depth, rgb & camera_info ———
        sub_d = Subscriber(self, Image, '/world/my_world/model/diffbot/link/base_link/sensor/depth_camera/depth_image')
        sub_r = Subscriber(self, Image, '/world/my_world/model/diffbot/link/base_link/sensor/rgb_camera/image')
        sub_i = Subscriber(self, CameraInfo, '/world/my_world/model/diffbot/link/base_link/sensor/depth_camera/camera_info')

        ats = ApproximateTimeSynchronizer([sub_d, sub_r, sub_i], queue_size=10, slop=0.1)
        ats.registerCallback(self.cb_images)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx, self.ry, self.rz = p.x, p.y, p.z
        self.ryaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.have_odom = True

    def cb_images(self, depth_msg, rgb_msg, info_msg):
        if not self.have_odom:
            self.get_logger().warn("Waiting for /odom_est…")
            return

        # CV conversion
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        annotated = rgb.copy()

        h, w = depth.shape
        fx, fy = info_msg.k[0], info_msg.k[4]
        cx, cy = info_msg.k[2], info_msg.k[5]

        # 1) HSV mask
        hsv  = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kern = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kern)
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask,'mono8'))

        # 2) find & filter contours
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f"Contours found: {len(cnts)}")
        cnts = [c for c in cnts if self.min_area <= cv2.contourArea(c) <= self.max_area]
        self.get_logger().info(f" After area≥{self.min_area}: {len(cnts)}")

        solid = []
        for c in cnts:
            a = cv2.contourArea(c)
            hull = cv2.convexHull(c)
            ha   = cv2.contourArea(hull)
            if ha>0 and (a/ha) >= self.min_solidity:
                solid.append(c)
        self.get_logger().info(f" After solidity≥{self.min_solidity}: {len(solid)}")

        # 3) back-project + feature check
        markers, new = MarkerArray(), False
        for c in solid:
            x,y,ww,hh = cv2.boundingRect(c)
            M = cv2.moments(c)
            if M['m00']==0: continue
            u = int(M['m10']/M['m00'])
            v = int(M['m01']/M['m00'])
            if not (0<=u<w and 0<=v<h): continue

            z = float(depth[v,u])
            if math.isnan(z) or z<=0 or z>5.0: continue

            # camera→world  
            x_cam = (u - cx)*z/fx
            y_cam = (v - cy)*z/fy
            cth,snt = math.cos(self.ryaw), math.sin(self.ryaw)
            wx = self.rx + (cth*x_cam - snt*y_cam)
            wy = self.ry + (snt*x_cam + cth*y_cam)
            wz = self.rz + z

            # ORB features (but min_features=0 so always passes)
            patch = rgb[y:y+hh, x:x+ww]
            kps, des = self.orb.detectAndCompute(patch, None)
            n_kps = 0 if kps is None else len(kps)
            self.get_logger().debug(f"Keypoints: {n_kps}")
            if n_kps < self.min_features:
                continue

            # new rock?
            is_new = True
            for e in self.db:
                dx,dy,dz = e['position'][0]-wx, e['position'][1]-wy, e['position'][2]-wz
                if math.hypot(dx,dy,dz) < self.new_thresh:
                    is_new = False
                    break

            if is_new:
                rid = len(self.db)
                self.db.append({
                    'id':          rid,
                    'position':    (wx,wy,wz),
                    'descriptors': des if des is not None else np.zeros((0,32),np.uint8)
                })
                new = True
                self.get_logger().info(f"New rock#{rid} @ ({wx:.2f},{wy:.2f},{wz:.2f})")

            # draw bounding box & kps
            col = (0,0,255) if is_new else (0,255,0)
            cv2.rectangle(annotated, (x,y), (x+ww,y+hh), col, 2)
            for kp in (kps or []):
                px,py = int(kp.pt[0])+x, int(kp.pt[1])+y
                cv2.circle(annotated, (px,py), 2, col, -1)

            # RViz marker
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns, m.id        = 'rocks', len(markers.markers)
            m.type, m.action  = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wx,wy,wz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = 1.0 if is_new else 0.0
            m.color.g = 0.0 if is_new else 1.0
            m.color.b, m.color.a = 0.0, 0.8
            markers.markers.append(m)

        # publish detections
        self.mkr_pub.publish(markers)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated,'bgr8'))

        if new:
            with open(self.db_path,'wb') as f:
                pickle.dump(self.db, f)
            self.get_logger().info(f"DB saved: {len(self.db)} rocks")


def main():
    rclpy.init()
    node = RockLocalizerBackproj()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

