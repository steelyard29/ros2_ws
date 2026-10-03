"""ROS2 H candidate telemetry, no TF or control publisher; no metric pose claim."""
import json
import math
import time
import cv2
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from h_target_fast import fast_candidates as candidates


class Preview(Node):
    def __init__(self):
        super().__init__('h_target_preview')
        cv2.setNumThreads(1)
        self.transport=self.declare_parameter('transport','raw').value
        if self.transport not in ('raw','compressed'):raise ValueError('raw or compressed transport required')
        self.bridge=CvBridge()
        self.pub=self.create_publisher(String,'/robocup/landing/h_candidate',10)
        topic='/robocup/downward/image_raw'+('/compressed' if self.transport=='compressed' else '')
        self.create_subscription(CompressedImage if self.transport=='compressed' else Image,
                                 topic,self.receive,qos_profile_sensor_data)
        self.history=deque(maxlen=5)
        self.last=0.;self.stamp=0;self.processed=0.;self.latest=None
        self.received=0;self.received_at=None;self.max_gap=0.;self.max_processing=0.
        self.create_timer(.1,self.tick)

    def receive(self,msg):
        now=time.monotonic()
        self.received+=1
        if self.received_at is not None:self.max_gap=max(self.max_gap,now-self.received_at)
        self.received_at=now
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp<=self.stamp or not -.05<=age<=.3:
            self.history.clear();self.latest=None
            return
        self.stamp=stamp
        if now-self.processed<.1:return
        self.processed=now
        try:
            image=(self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8') if self.transport=='compressed'
                   else self.bridge.imgmsg_to_cv2(msg,'bgr8'))
            result=candidates(image)
        except (ValueError,RuntimeError) as exc:
            self.get_logger().warning(str(exc));result=[]
        self.max_processing=max(self.max_processing,time.monotonic()-now)
        self.last=now
        if len(result)!=1:
            self.history.clear();self.latest=None
            return
        p=result[0]['center_px']
        if self.history and (now-self.history[-1][0]>.3 or math.dist(p,self.history[-1][1])>15):
            self.history.clear()
        self.history.append((now,p))
        self.latest={'source_stamp_ns':stamp,'frame_id':msg.header.frame_id,
                     'center_px':p,'image_size':[image.shape[1],image.shape[0]],
                     'error_from_image_center_px':[p[0]-image.shape[1]/2,p[1]-image.shape[0]/2],
                     'shape_distance':result[0]['shape_distance']}

    def tick(self):
        source_age=(self.get_clock().now().nanoseconds-self.stamp)/1e9
        fresh=time.monotonic()-self.last<=.3 and 0<=source_age<=.25
        stable=bool(fresh and self.latest and len(self.history)==5)
        data={'candidate_stable':stable,'metric_pose_valid':False,
              'landing_boundary_valid':False,'descent_allowed':False,
              'transport':self.transport,'received_frames':self.received,
              'maximum_receive_gap_s':self.max_gap,'maximum_processing_s':self.max_processing,
              'source_age_s':source_age}
        if fresh and self.latest:data.update(self.latest)
        self.pub.publish(String(data=json.dumps(data)))


if __name__=='__main__':
    rclpy.init();node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
