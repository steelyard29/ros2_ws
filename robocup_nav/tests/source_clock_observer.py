"""One deadline-bound, metadata-only stereo/VIO observer; no publishers."""
import argparse
import json
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--deadline',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 0<a.deadline-time.monotonic()<=20:p.error('expired/excessive deadline')
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from bench_session_deadline import DeadlineAlarm
    alarm=DeadlineAlarm(a.deadline);alarm.arm()
    report=dict(read_only=True,pixels_saved=False,px4_output=False,streams={},completed=False)
    rows={k:[] for k in ('infra1','infra2','vio')}
    node=None;ros=None
    try:
        import rclpy as ros
        from sensor_msgs.msg import Image
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        ros.init(domain_id=176)
        node=ros.create_node('source_clock_once',enable_rosout=False,
            start_parameter_services=False,use_global_arguments=False)
        def receive(key,m):
            now=time.monotonic()
            if now>=a.deadline-1:return
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            clock=node.get_clock().now().nanoseconds
            rows[key].append(dict(at=now,stamp_ns=stamp,ros_now_ns=clock,
                age_s=(clock-stamp)/1e9))
        for k in rows:
            topic=('/visual_slam/tracking/odometry' if k=='vio' else
                   '/camera/camera/'+k+'/image_rect_raw')
            node.create_subscription(Odometry if k=='vio' else Image,topic,
                lambda m,key=k:receive(key,m),qos_profile_sensor_data)
        while time.monotonic()<a.deadline-1:
            ros.spin_once(node,timeout_sec=.02)
        report['completed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        if node:node.destroy_node()
        if ros:ros.try_shutdown()
        alarm.cancel()
        for key,data in rows.items():
            ages=sorted(r['age_s'] for r in data)
            report['streams'][key]=dict(count=len(data),
                age_min_s=min(ages) if ages else None,
                age_median_s=ages[len(ages)//2] if ages else None,
                age_max_s=max(ages) if ages else None,
                future_over_50ms=sum(x<-.05 for x in ages),
                max_receipt_gap_s=max((b['at']-c['at'] for c,b in zip(data,data[1:])),default=None),
                samples=data)
        report['closed_monotonic']=time.monotonic()
        a.output.write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['completed'] and all(rows.values()) else 1


if __name__=='__main__':raise SystemExit(main())
