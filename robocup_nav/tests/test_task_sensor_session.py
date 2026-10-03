import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from task_sensor_session import validate_container,ROOT


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.c={'State':{'Running':False,'Status':'exited'},'Config':{
            'Entrypoint':['/usr/local/bin/scripts/deploy-entrypoint.sh'],'Cmd':['sleep','infinity']},
            'Mounts':[{'Source':str(ROOT.parent),'Destination':'/workspaces/ros2_ws'}]}

    def test_owned_idle_container(self):validate_container(self.c)

    def test_refuses_running_or_unknown_startup(self):
        for field,value in (('running',True),('command',['start_robot.sh']),('entry',['unknown']),('mount',[])):
            c=copy.deepcopy(self.c)
            if field=='running':c['State']['Running']=value
            elif field=='command':c['Config']['Cmd']=value
            elif field=='entry':c['Config']['Entrypoint']=value
            else:c['Mounts']=value
            with self.assertRaises(ValueError):validate_container(c)


if __name__=='__main__':unittest.main()
