"""Single-attempt container VIO process owner. Explicit sensor-only/isolated start.

Construction is inert. No camera/Agent startup, ROS publisher, real EV route or
automatic restart. Call stop after inhibiting the consuming EV owner; poll_close
is nonblocking so PX4 servicing need not wait for container cleanup.
"""
import os
from pathlib import Path
import subprocess
import time
import uuid
from disarmed_ev_lifecycle import check_receipt
from resident_reader_lifecycle import ReaderLeaseOwner
from resident_vio_transport import ResidentVioPort,ResidentVioPipe

ROOT=Path(__file__).resolve().parents[1]
CONTAINER_ROOT=Path('/workspaces/ros2_ws/robocup_nav')


def container_path(path):
    return str(CONTAINER_ROOT/Path(path).resolve().relative_to(ROOT.resolve()))


class ContainerResidentInputs:
    def __init__(self,out,*,isolated=False,sensor_only=False,domain=183,clock=time.monotonic,popen=subprocess.Popen):
        if isolated==sensor_only:raise ValueError('select exactly one explicit reader mode')
        if domain not in (range(180,188) if isolated else (176,)):
            raise ValueError('invalid reader domain for mode')
        self.isolated=isolated
        self.out=Path(out).resolve()
        self.out.relative_to((ROOT/'evidence').resolve())
        self.session=uuid.uuid4().hex;self.domain=domain;self.clock=clock;self.popen=popen
        self.receiver=ResidentVioPort(self.session,domain=domain,isolated=isolated,clock=clock)
        self.port=self.receiver.port
        self.control=self.out/'resident_reader.control.json'
        self.receipt=self.out/'resident_reader.closed.json'
        self.owner=None;self.process=None;self.pipe=None;self.log=None
        self.attempted=False;self.stopping=False;self.fault=None;self.cleanup=None
        self.last_renewal=None;self.deadline=None;self.stop_error=None
        self.exit_observed_at=None

    def start(self,deadline=None):
        if self.attempted or self.stopping:raise RuntimeError('resident reader is single-attempt')
        if self.isolated and (deadline is None or not 0<deadline-self.clock()<=25):
            raise ValueError('invalid isolated reader budget')
        if not self.isolated and deadline is not None:
            raise ValueError('sensor-only reader uses owner lease, not bench deadline')
        self.attempted=True;self.deadline=deadline
        try:
            # Exclusive directory prevents stale control/receipt reuse.
            self.out.mkdir(parents=False,exist_ok=False)
            self.owner=ReaderLeaseOwner(self.control,self.session,clock=self.clock)
            self.owner.renew();self.last_renewal=self.clock()
            self.log=(self.out/'resident_reader.stderr.log').open('x')
            command=['docker','exec','-e','ROS_LOCALHOST_ONLY=1','isaac_ros_dev']
            if self.isolated:command+=['timeout','--signal=TERM','--kill-after=1','26']
            command+=['bash','-c',
                'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; exec "$@"',
                'resident_reader','python3',str(CONTAINER_ROOT/'scripts/resident_vio_worker.py'),
                '--run-isolated' if self.isolated else '--lease-sensor-only',
                '--session',self.session,'--domain',str(self.domain),
                '--control',container_path(self.control),
                '--receipt',container_path(self.receipt)]
            if self.isolated:command+=['--deadline',str(deadline)]
            self.process=self.popen(command,stdout=subprocess.PIPE,stderr=self.log,stdin=subprocess.DEVNULL)
            self.pipe=ResidentVioPipe(self.process.stdout.fileno(),self.receiver)
        except Exception as exc:
            self.stop(str(exc))
            if self.process is None and self.log:self.log.close()
            raise

    def pump(self):
        if self.stopping:raise RuntimeError(self.fault or 'resident reader stopping')
        if self.process is None:raise RuntimeError('resident reader not started')
        try:
            if self.process.poll() is not None:raise RuntimeError('resident reader process ended')
            if self.deadline is not None and self.clock()>=self.deadline:
                raise TimeoutError('isolated resident deadline')
            self.pipe.pump()
            if self.receiver.graph_at is not None:
                for topic in self.receiver.port.reads:self.receiver.get_publishers_info_by_topic(topic)
            if self.clock()-self.last_renewal>=.2:
                if self.clock()-self.last_renewal>1.:
                    raise TimeoutError('resident owner loop stalled; no lease revival')
                self.owner.renew();self.last_renewal=self.clock()
        except Exception as exc:self.stop(str(exc));raise

    def stop(self,reason='owner requested stop'):
        if self.stopping:return
        self.stopping=True;self.fault=reason
        if self.pipe:self.pipe.close(reason)
        else:self.receiver.fault=reason
        if self.owner:
            try:self.owner.renew(stop=True)
            except Exception as exc:self.stop_error=repr(exc)

    def poll_close(self):
        if not self.stopping:raise RuntimeError('request stop before close polling')
        if self.cleanup is not None:return self.cleanup
        if self.process is None:
            self.cleanup=dict(confirmed=False,process_started=False,stop_error=self.stop_error)
            return self.cleanup
        # Drain without dispatching queued data after stop, never renew lease.
        try:
            os.set_blocking(self.process.stdout.fileno(),False)
            for _ in range(8):
                if not os.read(self.process.stdout.fileno(),4096):break
        except (BlockingIOError,InterruptedError):pass
        code=self.process.poll()
        if code is None:return dict(confirmed=False,running=True,stop_error=self.stop_error)
        if self.exit_observed_at is None:self.exit_observed_at=self.clock()
        # docker client exit can precede the container worker's atomic receipt.
        # Keep checking within a bounded grace period, without restarting it.
        if not self.receipt.exists() and self.clock()-self.exit_observed_at<.5:
            return dict(confirmed=False,running=False,awaiting_receipt=True,
                        client_exit_code=code,stop_error=self.stop_error)
        self.cleanup=check_receipt(self.receipt,self.session,code,self.clock())
        self.cleanup.update(running=False,stop_error=self.stop_error)
        self.process.stdout.close()
        if self.log:self.log.close()
        return self.cleanup
