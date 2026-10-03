"""Synthetic marker fields match vendor heightLimitToMarker; test data only."""
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from task_map_profile import mapper_profile


def markers(stamp,initial_z=0.):
    p=mapper_profile(initial_z)
    for kind,height in [('bottom_height_limit',p['center_z']-p['band_below']),
                        ('top_height_limit',p['center_z']+p['band_above'])]:
        m=Marker();m.header.stamp=stamp;m.header.frame_id='odom';m.ns=kind
        m.type=Marker.TRIANGLE_LIST;m.action=Marker.ADD;m.id=0
        m.scale.x=m.scale.y=m.scale.z=1.
        m.points=[Point(x=x,y=y,z=height) for x,y in
            [(1.,1.),(-1.,1.),(1.,-1.),(-1.,1.),(1.,-1.),(-1.,-1.)]]
        yield m
