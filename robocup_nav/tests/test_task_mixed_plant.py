"""No ROS context/network: actual mixed serializer to ideal feedback model."""
import math
from types import SimpleNamespace as NS
import unittest
from task_mixed_plant import MixedXY
from test_task_handoff import TaskRig


class MixedPlantTests(unittest.TestCase):
    def command(self,**changes):
        data=dict(position=[math.nan,math.nan,-.16],velocity=[.1,0.,math.nan],yaw=.8,timestamp=1)
        data.update(changes);return NS(**data)

    def test_mixed_motion_and_inverse_signs(self):
        m=MixedXY();m.accept(self.command(),1.)
        m.step(1.02,.02,armed=True,offboard=True,heartbeat_fresh=True)
        self.assertAlmostEqual(m.x,2.002);self.assertAlmostEqual(m.y,-3.)
        p,v=m.odom()
        self.assertAlmostEqual(p[0],.002*math.cos(.8))
        self.assertAlmostEqual(p[1],.002*math.sin(.8))
        self.assertAlmostEqual(v[0],.1*math.cos(.8))

    def test_actual_task_messages_preserve_odom_direction(self):
        from px4_msgs.msg import VehicleLocalPosition
        rig=TaskRig(hover=3.);rig.cruise()
        _,messages=rig.tick();msg=messages['trajectory_setpoint']
        m=MixedXY();m.accept(msg,1.)
        m.step(1.02,.02,armed=True,offboard=True,heartbeat_fresh=True)
        p,v=m.odom()
        self.assertGreater(p[0],0.);self.assertAlmostEqual(p[1],0.,places=8)
        self.assertGreater(v[0],0.)
        feedback=VehicleLocalPosition()
        feedback.x,feedback.y,feedback.vx,feedback.vy=m.x,m.y,m.vx,m.vy
        self.assertEqual(type(m.vx),float)
        self.assertAlmostEqual(feedback.vx,float(msg.velocity[0]))

    def test_no_stale_ground_land_or_heartbeat_motion(self):
        for flags in (dict(armed=False,offboard=True,heartbeat_fresh=True),
                      dict(armed=True,offboard=False,heartbeat_fresh=True),
                      dict(armed=True,offboard=True,heartbeat_fresh=False)):
            m=MixedXY();m.accept(self.command(),1.);m.step(1.02,.02,**flags)
            self.assertEqual((m.x,m.y),(2.,-3.))
        m=MixedXY();m.accept(self.command(),1.)
        m.step(1.251,.02,armed=True,offboard=True,heartbeat_fresh=True)
        self.assertEqual((m.vx,m.vy),(0.,0.))

    def test_invalid_setpoints_do_not_mutate_accepted_target(self):
        for changes in (dict(velocity=[.16,0.,math.nan]),dict(velocity=[.1,0.,0.]),
                        dict(position=[2.,math.nan,-.16]),dict(yaw=0.),
                        dict(position=[math.nan,math.nan,-.3])):
            m=MixedXY()
            with self.assertRaises(ValueError):m.accept(self.command(**changes),1.)
            self.assertEqual(m.stamp,0)
        m=MixedXY();m.accept(self.command(),1.)
        with self.assertRaises(ValueError):m.accept(self.command(),1.01)

    def test_original_vertical_target_still_supported(self):
        m=MixedXY()
        z=m.accept(self.command(position=[2.,-3.,.5],velocity=[math.nan]*3),1.)
        self.assertEqual(z,.5);self.assertEqual(m.target,(0.,0.))
