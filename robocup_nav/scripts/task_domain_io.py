"""Selective task I/O in a separate DDS context, not a topic/domain bridge.

Sensor/map/nav subscriptions and navigation goals stay in the perception domain.
No sensor retransmission into the PX4 domain and no /fmu writer in this module.
The single caller thread pumps both contexts; no concurrent core callbacks.
"""
import json
import time


def topic_sets(isolated=False):
    from task_scene_input import TaskSceneInput
    from flight_runtime import topics
    runtime,_=topics(isolated)
    reads={('/robocup/task_test/input/'+k if isolated else v)
           for k,v in TaskSceneInput.TOPICS.items()}
    if isolated:reads.discard('/robocup/task_test/input/vio')
    reads.update((runtime['vio'],runtime['tracking']))
    prefix='/robocup/task_test' if isolated else '/robocup/task'
    return reads,{prefix+'/navigation_goal',prefix+'/readiness'}


class ScopedPerceptionNode:
    def __init__(self,node,*,isolated=False):
        self._node=node;self.isolated=isolated
        self.reads,self.writes=topic_sets(isolated)
        self.subscriptions=[]

    @property
    def domain_id(self):return self._node.context.get_domain_id()

    def create_subscription(self,kind,topic,callback,qos):
        if topic not in self.reads:raise ValueError('unapproved perception subscription: '+topic)
        sub=self._node.create_subscription(kind,topic,callback,qos)
        self.subscriptions.append(sub)
        return sub

    def create_publisher(self,kind,topic,qos):
        if topic not in self.writes:raise ValueError('unapproved perception publisher: '+topic)
        return self._node.create_publisher(kind,topic,qos)

    def count_publishers(self,topic):
        if topic not in self.reads|self.writes:raise ValueError('unapproved perception graph query: '+topic)
        return self._node.count_publishers(topic)

    def get_publishers_info_by_topic(self,topic):
        if topic not in self.reads|self.writes:raise ValueError('unapproved perception graph query: '+topic)
        return self._node.get_publishers_info_by_topic(topic)

    def get_clock(self):return self._node.get_clock()


class PerceptionDomain:
    def __init__(self,*,domain_id=176,isolated=False):
        if domain_id not in ((180,181,182,183) if isolated else (176,)):
            raise ValueError('unexpected perception domain')
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        self.context=Context();self.node=None;self.executor=None
        try:
            self.context.init(args=[],domain_id=domain_id)
            self.node=Node('robocup_task_perception_io',context=self.context,
                use_global_arguments=False,enable_rosout=False,start_parameter_services=False)
            self.executor=SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.port=ScopedPerceptionNode(self.node,isolated=isolated)
        except Exception:
            self.close();raise

    def pump(self):
        # No extra thread: all scene/core callbacks serialize with PX4 callbacks.
        until=time.monotonic()+.004
        for _ in range(16):
            if time.monotonic()>=until:break
            self.executor.spin_once(timeout_sec=0.)

    def close(self):
        if self.executor:self.executor.shutdown(timeout_sec=.5)
        if self.node:self.node.destroy_node()
        self.context.try_shutdown()


class TrackingStatusRelay:
    """Retain vendor source timestamp, and latch unexpected source replacement."""
    def __init__(self):self.source=None;self.fault=None

    def encode(self,msg,publishers):
        if self.fault:return None
        if len(publishers)!=1:
            # Discovery absence is not a new fabricated tracking status.
            if len(publishers)>1:self.fault='multiple raw tracking publishers'
            return None
        source=bytes(publishers[0].endpoint_gid)
        if self.source is not None and source!=self.source:
            self.fault='raw tracking source replaced';return None
        self.source=source
        return json.dumps(dict(vo_state=int(msg.vo_state),
            stamp_ns=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)))
