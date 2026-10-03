"""Isolated ROS2 replay of recorded H image + blank/stale faults. No devices."""
import os
os.environ['ROS_DOMAIN_ID']='180'
os.environ['ROS_LOCALHOST_ONLY']='1'
import sys,json,time
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from h_target_preview import Preview


def main():
    rclpy.init();node=rclpy.create_node('h_replay_fixture');preview=Preview()
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(preview)
    compressed=preview.transport=='compressed'
    pub=node.create_publisher(CompressedImage if compressed else Image,
        '/robocup/downward/image_raw'+('/compressed' if compressed else ''),10)
    results=[]
    node.create_subscription(String,'/robocup/landing/h_candidate',lambda m:results.append(json.loads(m.data)),10)
    image=cv2.imread(str(ROOT/'evidence/downward_20260929_113856/image.png'))
    if image is None:raise RuntimeError('recorded H fixture missing')
    bridge=CvBridge()
    def phase(pixels,seconds):
        start=len(results);end=time.monotonic()+seconds;last=0
        while time.monotonic()<end:
            now=time.monotonic()
            if pixels is not None and now-last>.15:
                m=bridge.cv2_to_compressed_imgmsg(pixels) if compressed else bridge.cv2_to_imgmsg(pixels,'bgr8')
                m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='fixture_optical';pub.publish(m);last=now
            executor.spin_once(timeout_sec=.02)
        return results[start:]
    try:
        good=phase(image,4)
        stale=phase(None,1)
        blank=phase(np.full_like(image,255),1)
        report={'stable_detected':any(r['candidate_stable'] for r in good),
                'stale_inhibits':bool(stale) and not stale[-1]['candidate_stable'],
                'blank_inhibits':bool(blank) and not blank[-1]['candidate_stable'],
                'never_allows_descent':all(not r['descent_allowed'] for r in results),
                'no_flight_inputs':not any(n.startswith('/fmu/in/') for n,_ in node.get_topic_names_and_types()),
                'flight_validation':False}
        report['passed']=all(v for k,v in report.items() if k!='flight_validation')
        out=ROOT/'evidence'/time.strftime('h_preview_dds_%Y%m%d_%H%M%S');out.mkdir()
        (out/'report.json').write_text(json.dumps(report,indent=2));print(out,report)
        return 0 if report['passed'] else 1
    finally:
        executor.shutdown();node.destroy_node();preview.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
