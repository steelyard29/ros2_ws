"""Allowlisted navigation request protocol; no PX4 command topic or ROS startup.

Currently enabled only by the isolated domain183 candidate. Each request is
single-session, sequential, deadline checked and acknowledged; never replayed.
"""
import json
import math
from navigation_goal_link import GoalLink

MAX_REQUEST=65536


class NavigationRequestGuard:
    def __init__(self,session,topics,clock):
        self.session,self.topics,self.clock=session,set(topics),clock
        self.seq=0;self.fault=None;self.goal=GoalLink()

    def accept(self,p):
        if self.fault:raise ValueError(self.fault)
        try:
            if (not isinstance(p,dict) or p.get('session')!=self.session
                    or type(p.get('seq')) is not int or p['seq']!=self.seq+1
                    or type(p.get('at')) not in (int,float)
                    or not math.isfinite(p['at']) or not 0<=self.clock()-p['at']<=.25
                    or p.get('topic') not in self.topics
                    or not isinstance(p.get('data'),str) or len(p['data'].encode())>MAX_REQUEST):
                raise ValueError('navigation pipe session/sequence/topic/age/size mismatch')
            data=json.loads(p['data'])
            if not isinstance(data,dict):raise ValueError('navigation request requires JSON object')
            if p['topic'].endswith('/navigation_goal'):self.goal.select(data)
            self.seq=p['seq'];return p['topic'],p['data']
        except Exception as exc:self.fault=str(exc);raise
