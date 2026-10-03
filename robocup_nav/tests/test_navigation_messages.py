import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_frame_contract import SessionAlignment,build_shadow_messages


class MessageTests(unittest.TestCase):
    def call(self,stamp=123456):
        a=SessionAlignment([0,0,0],0,[3,4,5],0,'s',(0,)*5,verified=True)
        return build_shadow_messages(a,stamp,now=1,pose_at=1,command_at=1,
            position_odom=[0,0,.6],velocity_odom=[.1,.02,0],yaw_odom=.1,
            vio_session='s',reset_counters=(0,)*5,exclusive_writer_verified=True)

    def test_mixed_message_fields_without_vehicle_commands(self):
        out=self.call();self.assertEqual(set(out),{'offboard_control_mode','trajectory_setpoint'})
        mode=out['offboard_control_mode'];sp=out['trajectory_setpoint']
        self.assertTrue(mode.position);self.assertFalse(mode.velocity)
        self.assertFalse(mode.acceleration);self.assertFalse(mode.attitude)
        self.assertTrue(math.isnan(sp.position[0]));self.assertTrue(math.isnan(sp.position[1]))
        self.assertAlmostEqual(sp.position[2],4.4,places=5)
        self.assertAlmostEqual(sp.velocity[0],.1,places=6)
        self.assertAlmostEqual(sp.velocity[1],-.02,places=6)
        self.assertTrue(math.isnan(sp.velocity[2]))
        self.assertTrue(all(math.isnan(x) for x in sp.acceleration))
        self.assertTrue(all(math.isnan(x) for x in sp.jerk))
        self.assertAlmostEqual(sp.yaw,-.1,places=6)
        self.assertTrue(math.isnan(sp.yawspeed))
        self.assertEqual(mode.timestamp,sp.timestamp)

    def test_invalid_timestamp(self):
        for value in (0,-1,True,1.5):
            with self.assertRaises(ValueError):self.call(value)


if __name__=='__main__':unittest.main()
