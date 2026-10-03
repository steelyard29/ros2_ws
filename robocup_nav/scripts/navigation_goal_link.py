"""Goal -> A* path -> APF command correlation. No ROS or hardware authority."""
from dataclasses import dataclass
import math
from navigation_limits import horizontal_velocity


@dataclass(frozen=True)
class LinkedCommand:
    goal_id: str
    velocity_xy: tuple
    stamp_ns: int
    path_stamp_ns: int


class GoalLink:
    def __init__(self):
        self.goal = None
        self.request_stamp = 0
        self.path_stamp = 0
        self.last_command_stamp = 0

    def select(self,goal):
        if (not isinstance(goal,dict) or goal.get('frame_id')!='odom'
                or not isinstance(goal.get('goal_id'),str) or not goal['goal_id']
                or len(goal.get('position',()))!=3
                or not all(type(v) in (int,float) and math.isfinite(v) for v in goal['position'])
                or type(goal.get('yaw')) not in (int,float) or not math.isfinite(goal['yaw'])):
            raise ValueError('invalid explicit odom navigation goal')
        bound=(goal['goal_id'],tuple(goal['position']),goal['yaw'])
        if self.goal is not None and self.goal[0]==bound[0] and self.goal!=bound:
            raise ValueError('goal ID reused with changed coordinates')
        if self.goal!=bound:
            self.goal=bound;self.path_stamp=0;self.request_stamp=0;self.last_command_stamp=0
        return bound

    def request(self,stamp_ns):
        if self.goal is None or type(stamp_ns) is not int or stamp_ns<=self.request_stamp:
            raise ValueError('missing goal or non-advancing planning request')
        self.request_stamp=stamp_ns
        return (self.goal[0],stamp_ns)

    def accept_path(self,ticket,points,now_ns):
        if self.goal is None or ticket!=(self.goal[0],self.request_stamp):return False
        if (type(now_ns) is not int or not 0<=now_ns-ticket[1]<=500_000_000
                or not points or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points)
                or math.dist(points[-1],self.goal[1][:2])>.08):return False
        self.path_stamp=ticket[1]
        return True

    def command(self,data,now_ns):
        if not isinstance(data,dict) or type(now_ns) is not int:return None
        if self.goal is None or not self.path_stamp:return None
        if data.get('schema')!=1 or data.get('valid') is not True or data.get('frame_id')!='odom':return None
        if data.get('path_stamp_ns')!=self.path_stamp:return None
        for key,limit in (('stamp_ns',250_000_000),('odom_stamp_ns',250_000_000),
                          ('scan_stamp_ns',250_000_000),('path_stamp_ns',500_000_000)):
            stamp=data.get(key)
            if type(stamp) is not int or stamp<=0 or not 0<=now_ns-stamp<=limit:return None
        if data['stamp_ns']<=self.last_command_stamp:return None
        try:velocity=horizontal_velocity(data.get('velocity_xy'))
        except (ValueError,TypeError,OverflowError):return None
        self.last_command_stamp=data['stamp_ns']
        return LinkedCommand(self.goal[0],tuple(velocity),data['stamp_ns'],self.path_stamp)
