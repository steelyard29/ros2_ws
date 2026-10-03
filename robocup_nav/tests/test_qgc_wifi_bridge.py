import ipaddress,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from qgc_wifi_bridge import GroundStation,common


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.g=GroundStation(ipaddress.ip_network('192.168.43.0/24'),'192.168.43.228')
        self.m=common.MAVLink(None,srcSystem=255,srcComponent=190)
        self.h=self.m.heartbeat_encode(common.MAV_TYPE_GCS,common.MAV_AUTOPILOT_INVALID,0,0,0).pack(self.m)

    def test_heartbeat_forwarded_without_modification(self):
        peer=('192.168.43.12',14550)
        self.assertEqual(self.g.receive(self.h,peer,0),[self.h])
        self.assertEqual(self.g.active(1),peer)
        self.assertIsNone(self.g.active(11))
        self.assertEqual(self.g.receive(self.h,('192.168.43.13',14550),12),[self.h])

    def test_unpaired_garbage_and_foreign_subnet_rejected(self):
        self.assertEqual(self.g.receive(b'garbage',('192.168.43.12',14550),0),[])
        self.assertEqual(self.g.receive(self.h,('10.0.0.2',14550),0),[])
        self.assertEqual(self.g.receive(self.h,('192.168.43.228',14550),0),[])
        self.g.receive(self.h,('192.168.43.12',14550),0)
        self.assertEqual(self.g.receive(self.h,('192.168.43.13',14550),1),[])


if __name__=='__main__':unittest.main()
