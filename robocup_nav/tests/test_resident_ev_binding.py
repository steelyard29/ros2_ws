import tempfile
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_ev_binding import ResidentEvOwner
from resident_vio_transport import ResidentVioPort


class BindingTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        p=patch('resident_ev_binding.LOCK_PATH',Path(tmp.name)/'owner.lock')
        p.start();self.addCleanup(p.stop)
        self.owner=ResidentEvOwner('s');self.addCleanup(self.owner.close)

    def test_exclusive_lock_and_no_reacquisition(self):
        self.owner.acquire();other=ResidentEvOwner('other');self.addCleanup(other.close)
        with self.assertRaises(BlockingIOError):other.acquire()
        self.owner.close()
        with self.assertRaises(RuntimeError):self.owner.acquire()
        fresh=ResidentEvOwner('new');self.addCleanup(fresh.close);fresh.acquire()

    def test_same_session_real_transport_required(self):
        self.owner.acquire();port=ResidentVioPort('s').port
        self.owner.validate('s',port)
        with self.assertRaises(ValueError):self.owner.validate('wrong',port)
        with self.assertRaises(ValueError):self.owner.validate('s',ResidentVioPort('wrong').port)
        with self.assertRaises(ValueError):
            self.owner.validate('s',ResidentVioPort('s',domain=183,isolated=True).port)
        port._node.fault='stopped'
        with self.assertRaises(ValueError):self.owner.validate('s',port)

    def test_closed_owner_rejects(self):
        self.owner.acquire();self.owner.close()
        with self.assertRaises(ValueError):self.owner.validate('s',ResidentVioPort('s').port)


if __name__=='__main__':unittest.main()
