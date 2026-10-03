"""Synthetic moving inputs for transport integration, NOT perception accuracy.

No ROS at import. H pixels use the same nominal camera model as reprojection.
Navigation is explicitly ideal goal-seeking, not an A*/APF substitute or score.
"""
import math
import numpy as np
import cv2


def checked_pose(record,session,now):
    if (record.get('session')!=session or not math.isfinite(record['at'])
            or not 0<=now-record['at']<=.15 or type(record.get('stamp_ns')) is not int
            or record['stamp_ns']<=0):
        raise ValueError('synthetic pose session/time mismatch')
    p=np.asarray(record['position'],float);v=np.asarray(record['velocity'],float)
    if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(v).all():
        raise ValueError('synthetic pose requires finite position and velocity')
    return p,v


def h_observation(position,camera,stamp_ns):
    p=np.asarray(position,float)
    optical=np.asarray(camera['body_from_optical'],float).T@(
        np.array([0.,0.,-.14])-p-np.asarray(camera['offset_flu_m'],float))
    result=dict(candidate_stable=False,source_stamp_ns=stamp_ns,frame_id=camera['frame_id'],
                image_size=camera['image_size'],synthetic=True)
    if not np.isfinite(optical).all() or optical[2]<=0:return result
    pixel,_=cv2.projectPoints(optical.reshape(1,3),np.zeros(3),np.zeros(3),
        np.asarray(camera['k'],float),np.asarray(camera['distortion'],float))
    pixel=pixel.reshape(2);w,h=camera['image_size']
    if np.isfinite(pixel).all() and 0<=pixel[0]<w and 0<=pixel[1]<h:
        result.update(candidate_stable=True,center_px=pixel.tolist())
    return result


def ideal_navigation(position,goal,stamp_ns):
    if not goal:return dict(synthetic=True,candidate_stable=False)
    target=np.asarray(goal['position'],float)
    if (target.shape!=(3,) or not np.isfinite(target).all() or goal.get('frame_id')!='odom'
            or not isinstance(goal.get('goal_id'),str) or not goal['goal_id']):
        raise ValueError('invalid synthetic navigation goal')
    velocity=.6*(target[:2]-np.asarray(position)[:2])
    norm=float(np.linalg.norm(velocity))
    if norm>.15:velocity*=.15/norm
    return dict(schema=1,frame_id='odom',goal_id=goal['goal_id'],stamp_ns=stamp_ns,
                path_stamp_ns=stamp_ns,velocity_xy=velocity.tolist(),synthetic_ideal_navigation=True)


def messages(record,session,now,camera,goal):
    """Construct actual ROS messages without any context, publisher or clock."""
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from nvblox_msgs.msg import DistanceMapSlice
    from task_map_fixture import markers
    import json
    p,v=checked_pose(record,session,now);ns=record['stamp_ns']
    od=Odometry();od.header.stamp.sec,od.header.stamp.nanosec=divmod(ns,10**9)
    od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.orientation.w=1.
    od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z=map(float,p)
    od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.linear.z=map(float,v)
    depth=Image();depth.header=od.header;depth.width=depth.height=1
    depth.encoding='16UC1';depth.step=2;depth.data=[208,7]
    grid=DistanceMapSlice();grid.header=od.header;grid.width=160;grid.height=100
    grid.resolution=.05;grid.origin.x=-1.8;grid.origin.y=-2.5;grid.origin.z=.86
    grid.unknown_value=-1000.;grid.data=[2.]*16000
    return dict(vio=od,depth=depth,map=grid,bounds=list(markers(od.header.stamp)),
        h=String(data=json.dumps(h_observation(p,camera,ns))),
        navigation=String(data=json.dumps(ideal_navigation(p,goal,ns))),
        tracking=String(data=json.dumps(dict(vo_state=1,stamp_ns=ns,synthetic=True))))
