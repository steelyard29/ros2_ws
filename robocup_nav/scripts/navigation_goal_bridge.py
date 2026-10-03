"""Actual Nav2 ComputePathToPose -> APF -> tagged candidate bridge; NO PX4.

Does not start sensors/controllers, change modes, or arm. Receives current
task goal JSON, rejects late prior-goal action results, and never retags a
plain unstamped Twist as a current navigation command.
"""
import json
import math
import time
from navigation_goal_link import GoalLink


def make_node():
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    from nav_msgs.msg import Path
    from std_msgs.msg import String

    class Bridge(Node):
        def __init__(self):
            super().__init__('robocup_navigation_goal_bridge',use_global_arguments=False)
            self.link=GoalLink();self.goal=None;self.pending=None;self.last_request=0.
            self.goal_seen=-1.
            self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
            self.path_pub=self.create_publisher(Path,'/robocup/legacy_apf/path',10)
            self.command_pub=self.create_publisher(String,'/robocup/task/navigation_command',10)
            self.create_subscription(String,'/robocup/task/navigation_goal',self.select,10)
            self.create_subscription(String,'/robocup/legacy_apf/candidate',self.command,10)
            self.create_timer(.05,self.tick)

        def select(self,msg):
            try:
                data=json.loads(msg.data);old=self.link.goal
                self.link.select(data);self.goal=data
                self.goal_seen=time.monotonic()
                if old!=self.link.goal:
                    self.pending=None;self.clear_path()
            except (ValueError,TypeError,KeyError):
                self.goal=None;self.link=GoalLink();self.pending=None;self.clear_path()

        def clear_path(self):
            self.link.path_stamp=0
            msg=Path();msg.header.frame_id='odom';msg.header.stamp=self.get_clock().now().to_msg()
            self.path_pub.publish(msg)

        def tick(self):
            if self.goal is None:return
            now=time.monotonic()
            if (not 0<=now-self.goal_seen<=.5
                    or self.count_publishers('/robocup/task/navigation_goal')!=1
                    or self.count_publishers('/robocup/legacy_apf/candidate')!=1
                    or self.count_publishers('/robocup/legacy_apf/path')!=1
                    or self.count_publishers('/robocup/task/navigation_command')!=1):
                self.clear_path();return
            if self.pending and now-self.last_request<.5:return
            if now-self.last_request<.25 or not self.client.server_is_ready():return
            stamp=self.get_clock().now().nanoseconds
            ticket=self.link.request(stamp);self.pending=ticket;self.last_request=now
            request=ComputePathToPose.Goal();request.planner_id='GridBased';request.use_start=False
            request.goal.header.frame_id='odom'
            request.goal.header.stamp=self.get_clock().now().to_msg()
            p=self.goal['position'];request.goal.pose.position.x=float(p[0]);request.goal.pose.position.y=float(p[1])
            request.goal.pose.position.z=float(p[2]);q=request.goal.pose.orientation
            q.z=math.sin(self.goal['yaw']/2);q.w=math.cos(self.goal['yaw']/2)
            self.client.send_goal_async(request).add_done_callback(lambda f:self.accepted(f,ticket))

        def accepted(self,future,ticket):
            try:
                handle=future.result()
                if ticket!=self.pending or not handle.accepted:
                    if handle.accepted:handle.cancel_goal_async()
                    elif ticket==self.pending:
                        self.pending=None;self.clear_path()
                    return
                handle.get_result_async().add_done_callback(lambda f:self.result(f,ticket))
            except Exception as exc:self.get_logger().error(str(exc))

        def result(self,future,ticket):
            if ticket!=self.pending:return
            self.pending=None
            try:
                wrapper=future.result();path=wrapper.result.path
                points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
                if (wrapper.status!=4 or path.header.frame_id!='odom'
                        or not self.link.accept_path(ticket,points,self.get_clock().now().nanoseconds)):
                    self.clear_path();return
                path.header.stamp.sec=ticket[1]//10**9;path.header.stamp.nanosec=ticket[1]%10**9
                self.path_pub.publish(path)
            except Exception as exc:
                self.clear_path();self.get_logger().error(str(exc))

        def command(self,msg):
            try:
                if (not 0<=time.monotonic()-self.goal_seen<=.5
                        or self.count_publishers('/robocup/legacy_apf/candidate')!=1):return
                linked=self.link.command(json.loads(msg.data),self.get_clock().now().nanoseconds)
                if linked is None:return
                out=String();out.data=json.dumps(dict(schema=1,goal_id=linked.goal_id,frame_id='odom',
                    stamp_ns=linked.stamp_ns,path_stamp_ns=linked.path_stamp_ns,
                    velocity_xy=linked.velocity_xy))
                self.command_pub.publish(out)
            except (ValueError,TypeError,KeyError):return
    return Bridge()


if __name__=='__main__':
    import rclpy
    rclpy.init(args=[]);node=make_node()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
