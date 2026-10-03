"""Read-only two-topic port for the resident EV owner.

No ROS imports, publishers, navigation goals, task lifetime, or camera processing
dependencies. The caller owns the underlying node/transport and its servicing.
"""


class ScopedVioInput:
    def __init__(self, node, *, isolated=False):
        self._node=node
        self.isolated=isolated
        prefix='/robocup/runtime_test/input/'
        self.reads=({prefix+'vio',prefix+'tracking'} if isolated else {
            '/visual_slam/tracking/odometry','/robocup/alignment/tracking'})
        self.subscriptions=[]

    @property
    def domain_id(self):
        return self._node.context.get_domain_id()

    def create_subscription(self,kind,topic,callback,qos):
        if topic not in self.reads:raise ValueError('unapproved resident VIO subscription')
        sub=self._node.create_subscription(kind,topic,callback,qos)
        self.subscriptions.append(sub)
        return sub

    def get_publishers_info_by_topic(self,topic):
        if topic not in self.reads:raise ValueError('unapproved resident VIO graph query')
        return self._node.get_publishers_info_by_topic(topic)
