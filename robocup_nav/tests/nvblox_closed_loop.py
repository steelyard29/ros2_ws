"""Actual Nav2 + candidate messages + simple XY/yaw plant, isolated domain176.

Not PX4 SITL: perfect pose/map, hypothetical response, no wind/thrust/altitude
dynamics. Do not infer measured braking or flight readiness from this test.
"""
from nvblox_navigation_test import *
from collections import deque,Counter
from dwb_msgs.msg import LocalPlanEvaluation
from rosidl_runtime_py.convert import message_to_ordereddict
from stopping_space import select_stopping_safe_velocity
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path as NavPath


class MovingFixture(NvbloxFixture):
    def __init__(self,controller='dwb'):
        self.controller=controller
        self.xy=np.array([1.,2.5]);self.v=np.zeros(2);self.yaw=0.
        self.desired_v=np.zeros(2);self.desired_yaw=0.
        self.z=0.;self.vz=0.;self.desired_z=0.
        self.scan_beams=180
        self.command_at=0.;self.plant_at=time.monotonic();self.samples=[]
        self.delayed=deque();self.contact=False
        self.reasons=Counter();self.max_input_x=0.;self.max_input_yaw=0.;self.max_target_v=0.
        self.speed_scales=Counter()
        super().__init__(trial=False)
        self.scan_pub=self.create_publisher(LaserScan,'/robocup/legacy_apf/scan',10)
        self.path_pub=self.create_publisher(NavPath,'/robocup/legacy_apf/path',10)
        self.create_subscription(Twist,'/robocup/legacy_apf/cmd_vel_odom',self.legacy_command,10)
        self.evaluations=[];self.raw_commands=[]
        self.navigation_sample=None;self.navigation_received_at=-1.
        self.control_timer=self.create_timer(.05,self.control_tick)
        self.create_subscription(LocalPlanEvaluation,'/controller_server/FollowPath/evaluation',self.evaluation,10)
        self.create_subscription(Twist,'/robocup/nvblox/cmd_vel_unchecked',
            lambda m:self.raw_commands.append([time.monotonic(),m.linear.x,m.angular.z]),10)
        def clear(p,v,c):
            if self.map_msg is None:return False
            free=np.asarray(self.map_msg.data).reshape(120,120)>0
            return stopping_space_clear(free,.05,(0,0),p,v,c,body_radius=.43,
                uncertainty=.05,latency=.3,braking=.2)
        mission=MissionShadow(make_band(initial_z=-.6),(.5,5.5,.5,5.5),clear)
        self.pipeline=ShadowPipeline(mission,SessionAlignment([1,2.5,0],0,[4,5,6],0,
                                     'synthetic',(0,)*5,verified=True),0.)

    def map(self,blocked=False):
        super().map(blocked)
        if getattr(self,'scene_name','wall')=='two-boxes' and not blocked:
            a=np.full((120,120),2.,dtype=float)
            a[0,:]=a[-1,:]=a[:,0]=a[:,-1]=-.01
            # 0.8m square boxes, 1.2m inner gap; nearest face 1.2m
            # in front of the synthetic initial center. Not a live map.
            a[22:38,44:60]=-.01;a[62:78,44:60]=-.01
            self.map_msg.data=a.ravel().tolist()

    def evaluation(self,msg):
        # Bounded snapshots, including every candidate's full critic scores.
        if len(self.evaluations)<3:self.evaluations.append(message_to_ordereddict(msg))
        elif len(self.evaluations)==3:self.evaluations.append(message_to_ordereddict(msg))
        else:self.evaluations[-1]=message_to_ordereddict(msg)

    def checked(self,msg):
        if self.controller!='dwb':return
        self.consume(msg,False)

    def legacy_command(self,msg):
        if self.controller in ('legacy-apf','path-apf'):
            self.raw_commands.append([time.monotonic(),msg.linear.x,msg.linear.y,msg.angular.z])
            self.consume(msg,True)

    def consume(self,msg,world_frame):
        self.navigation_sample=(msg,world_frame)
        self.navigation_received_at=time.monotonic();self.last_checked=msg
        self.max_input_x=max(self.max_input_x,abs(msg.linear.x));self.max_input_yaw=max(self.max_input_yaw,abs(msg.angular.z))
        self.checked_nonzero+=int(abs(msg.linear.x)+abs(msg.angular.z)>.001)

    def control_tick(self):
        if self.navigation_sample is None:return
        msg,world_frame=self.navigation_sample;now=time.monotonic()
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        world=(msg.linear.x,msg.linear.y) if world_frame else (c*msg.linear.x-s*msg.linear.y,s*msg.linear.x+c*msg.linear.y)
        scene=self.make_scene(now,world)
        r=self.run_pipeline(now,scene,msg.angular.z)
        if r.get('fault'):self.pipeline_fault=r['fault']
        self.reasons[r.get('intention',{}).get('reason',r.get('fault','none'))]+=1
        if r['messages']:
            sp=r['messages']['trajectory_setpoint']
            self.delayed.append((now+.1,np.array([sp.velocity[0],-sp.velocity[1]]),-sp.yaw,6-sp.position[2]))
            self.pipeline_messages+=1
            self.max_target_v=max(self.max_target_v,math.hypot(*sp.velocity[:2]))
        else:
            # Terminal decisions must fence queued old commands too. This is
            # a synthetic plant stop assumption, not a measured PX4 response.
            self.delayed.clear();self.desired_v[:]=0;self.desired_z=self.z

    def make_scene(self,now,world):
        if self.controller=='path-apf':
            # New candidate selection, same final map-backed stopping rule.
            # Do not shrink footprint/uncertainty to force a doorway pass.
            world,scale=select_stopping_safe_velocity(world,
                lambda c:self.pipeline.mission.stopping_check(self.xy,self.v,c))
            self.speed_scales[str(scale)]+=1
        return Scene(at=now,position=(*self.xy,self.z),velocity_xy=tuple(self.v),tilt_rad=0.,
            localization_ok=True,airborne=True,map_at=now,footprint_known_free=True,
            navigation_at=self.navigation_received_at,navigation_velocity_odom=world)

    def run_pipeline(self,now,scene,yaw_rate):
        return self.pipeline.step(now,scene,timestamp_us=self.get_clock().now().nanoseconds//1000,
            vio_session='synthetic',reset_counters=(0,)*5,exclusive_writer_verified=True,yaw_rate_odom=yaw_rate)

    def advance_vertical(self,dt):
        pass

    def tick(self):
        now=time.monotonic();dt=now-self.plant_at;self.plant_at=now
        while self.delayed and self.delayed[0][0]<=now:
            _,self.desired_v,self.desired_yaw,self.desired_z=self.delayed.popleft();self.command_at=now
        if now-self.command_at>.25:self.desired_v[:]=0
        accel=(self.desired_v-self.v)/.3;norm=np.linalg.norm(accel)
        if norm>.2:accel*=.2/norm
        self.v+=accel*dt;self.xy+=self.v*dt
        self.advance_vertical(dt)
        error=math.atan2(math.sin(self.desired_yaw-self.yaw),math.cos(self.desired_yaw-self.yaw))
        self.yaw+=float(np.clip(error/.3,-.2,.2))*dt
        yaw_velocity=float(np.clip(error/.3,-.2,.2))
        stamp=self.get_clock().now().to_msg()
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y=map(float,self.xy)
        odom.pose.pose.position.z=self.z;odom.twist.twist.linear.z=self.vz
        odom.pose.pose.orientation.z=math.sin(self.yaw/2);odom.pose.pose.orientation.w=math.cos(self.yaw/2)
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        odom.twist.twist.linear.x=float(c*self.v[0]+s*self.v[1]);odom.twist.twist.linear.y=float(-s*self.v[0]+c*self.v[1])
        odom.twist.twist.angular.z=yaw_velocity
        self.raw_odom_pub.publish(odom)
        if self.controller in ('legacy-apf','path-apf') and self.map_msg is not None:
            # Ideal synthetic 360deg FLU scan of the same binary obstacle map.
            # Not evidence of real scanner extrinsics, range noise or coverage.
            a=np.asarray(self.map_msg.data).reshape(120,120)>0
            angles=np.arange(self.scan_beams)*2*math.pi/self.scan_beams-math.pi+self.yaw
            distances=np.arange(.1,8.,.025)
            xs=self.xy[0]+np.cos(angles)[:,None]*distances
            ys=self.xy[1]+np.sin(angles)[:,None]*distances
            ix=np.floor(xs/.05).astype(int);iy=np.floor(ys/.05).astype(int)
            valid=(ix>=0)&(ix<120)&(iy>=0)&(iy<120)
            hit=~(valid&a[np.clip(iy,0,119),np.clip(ix,0,119)])
            ranges=distances[np.argmax(hit,axis=1)]
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/self.scan_beams
            scan.angle_max=scan.angle_min+(self.scan_beams-1)*scan.angle_increment
            scan.range_min=.05;scan.range_max=8.;scan.ranges=ranges.astype(np.float32).tolist()
            self.scan_pub.publish(scan)
        status=VisualSlamStatus();status.header.stamp=stamp;status.vo_state=1;self.vo_pub.publish(status)
        depth=Image();depth.header.stamp=stamp;depth.header.frame_id='camera_depth_optical_frame'
        depth.width=depth.height=1;depth.step=2;depth.encoding='16UC1';depth.data=b'\xd0\x07';self.depth_pub.publish(depth)
        if self.map_msg is not None:
            self.map_msg.header.stamp=stamp;self.slice_pub.publish(self.map_msg)
            free=np.asarray(self.map_msg.data).reshape(120,120)>0
            self.contact |= not stopping_space_clear(free,.05,(0,0),self.xy,(0,0),(0,0),
                body_radius=.43,uncertainty=0,latency=0,braking=1)
        self.samples.append([now,*map(float,self.xy),self.yaw,*map(float,self.v)])


def main(fixture_type=MovingFixture,full_landing=False):
    parser=argparse.ArgumentParser()
    parser.add_argument('--sim-time',type=float,default=None)
    parser.add_argument('--generator',choices=['LimitedAccelGenerator','StandardTrajectoryGenerator'],default='LimitedAccelGenerator')
    parser.add_argument('--controller',choices=['dwb','legacy-apf','path-apf'],default='legacy-apf' if full_landing else 'dwb')
    parser.add_argument('--scene',choices=['wall','two-boxes'],default='wall')
    parser.add_argument('--scan-beams',type=int,choices=(180,720),default=180)
    parser.add_argument('--initial-y-offset',type=float,default=0.)
    args=parser.parse_args()
    if not math.isfinite(args.initial_y_offset) or abs(args.initial_y_offset)>.1:
        parser.error('synthetic starting lateral offset must be within +/-0.1m')
    out=ROOT/'evidence'/time.strftime('nvblox_closed_loop_%Y%m%d_%H%M%S');out.mkdir(parents=True)
    report={'passed':False,'flight_validation':False,'px4_sitl':False,
            'test_sim_time_override':args.sim_time,'test_generator':args.generator,
            'controller':args.controller,'scene':args.scene,
            'synthetic_scan_beams':args.scan_beams,'initial_y_offset_m':args.initial_y_offset};proc=None;apf_proc=None
    rclpy.init();fixture=fixture_type(args.controller);log=open(out/'navigation.log','w')
    fixture.scene_name=args.scene
    fixture.scan_beams=args.scan_beams;fixture.xy[1]+=args.initial_y_offset
    fixture.pipeline.alignment=SessionAlignment([*fixture.xy,0.],0,[4,5,6],0,'synthetic',(0,)*5,verified=True)
    try:
        proc=subprocess.Popen(['ros2','launch',str(ROOT/'launch/nvblox_navigation.launch.py'),'trial_profile:=true','debug_trajectories:=true',
                               f'debug_sim_time:={args.sim_time or 0}',f'debug_generator:={args.generator}'],
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        fixture.map()
        for name in ('planner_server','controller_server','bt_navigator'):
            client=fixture.create_client(GetState,f'/{name}/get_state');fixture.wait(client.service_is_ready,35)
            deadline=time.monotonic()+35
            while fixture.result(client.call_async(GetState.Request())).current_state.id!=3:
                if time.monotonic()>deadline:raise RuntimeError(name+' inactive')
                fixture.spin_for(.5)
        report['evaluation_topics']=[n for n,t in fixture.get_topic_names_and_types()
                                     if 'dwb_msgs/msg/LocalPlanEvaluation' in t]
        for topic in report['evaluation_topics']:
            if topic!='/controller_server/FollowPath/evaluation':
                fixture.create_subscription(LocalPlanEvaluation,topic,fixture.evaluation,10)
        fixture.wait(fixture.plan.server_is_ready);path=fixture.planned_path()
        assert path.status==4
        (out/'planned_path.json').write_text(json.dumps(message_to_ordereddict(path.result.path)))
        result=None
        if args.controller=='dwb':
            fixture.wait(fixture.follow.server_is_ready)
            goal=FollowPath.Goal();goal.path=path.result.path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
            handle=fixture.result(fixture.follow.send_goal_async(goal));assert handle.accepted
            result=handle.get_result_async()
        else:
            env=dict(os.environ)
            # The install marker is an absolute /home/cfly symlink, unavailable
            # in this container. Use the real generated build index instead.
            env['AMENT_PREFIX_PATH']=str(ROOT.parent/'build/uav_task/ament_cmake_index')+':'+env.get('AMENT_PREFIX_PATH','')
            apf_proc=subprocess.Popen([str(ROOT/'build/legacy_apf_shadow/legacy_apf_shadow'),
              '--ros-args','-p','controller_config_file:='+str(ROOT/'config/legacy_apf_shadow.yaml'),
              '-p','path_guided:='+str(args.controller=='path-apf').lower(),
              '--log-level','warn'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            fixture.wait(lambda:fixture.path_pub.get_subscription_count()>0,10)
            fixture.path_pub.publish(path.result.path)
        observation_s=getattr(fixture,'test_observation_seconds',85.)
        if not 0 < observation_s <= 120.:
            raise ValueError('offline observation window outside bounded test limits')
        report['offline_observation_window_s']=observation_s
        deadline=time.monotonic()+observation_s
        goal_since=None
        def finished():
            nonlocal goal_since
            if full_landing:return fixture.landing_state=='DONE'
            if result is not None:return result.done()
            settled=np.linalg.norm(fixture.xy-[5.,2.5])<.10 and np.linalg.norm(fixture.v)<.03
            goal_since=(time.monotonic() if goal_since is None else goal_since) if settled else None
            return goal_since is not None and time.monotonic()-goal_since>=2.
        while not finished() and time.monotonic()<deadline and not fixture.contact:
            rclpy.spin_once(fixture,timeout_sec=.05)
        report.update(final_xy=fixture.xy.tolist(),goal_error_m=float(np.linalg.norm(fixture.xy-[5.,2.5])),
            conservative_envelope_contact=bool(fixture.contact),pipeline_fault=fixture.pipeline_fault,
            mission_state=fixture.pipeline.mission.state,candidate_messages=fixture.pipeline_messages,
            action_status=result.result().status if result is not None and result.done() else None,
            legacy_goal_reached=bool(finished()) if result is None else None,
            max_guarded_forward=fixture.max_input_x,max_guarded_yaw_rate=fixture.max_input_yaw,
            max_candidate_speed=fixture.max_target_v,pipeline_reasons=dict(fixture.reasons),
            candidate_speed_scale_counts=dict(fixture.speed_scales),
            final_speed_mps=float(np.linalg.norm(fixture.v)),
            goal_settle_required_s=2. if not full_landing and args.controller!='dwb' else None,
            no_flight_inputs=not any(n.startswith('/fmu/in/') for n,_ in fixture.get_topic_names_and_types()))
        if result is not None and not result.done():fixture.result(handle.cancel_goal_async())
        report['passed']=bool((report['action_status']==4 or report['legacy_goal_reached']) and report['goal_error_m']<.2 and
            not fixture.contact and not fixture.pipeline_fault and report['no_flight_inputs'])
        if full_landing:
            report.update(fixture.landing_report())
            report['passed'] &= fixture.landing_state=='DONE' and report.get('sortie_validation_passed',True)
    except Exception as exc:report['error']=str(exc)
    finally:
        if apf_proc is not None and apf_proc.poll() is None:
            os.killpg(apf_proc.pid,signal.SIGINT)
            try:apf_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(apf_proc.pid,signal.SIGTERM);apf_proc.wait(timeout=5)
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        (out/'trajectory.json').write_text(json.dumps(fixture.samples))
        (out/'evaluations.json').write_text(json.dumps(fixture.evaluations))
        (out/'raw_commands.json').write_text(json.dumps(fixture.raw_commands))
        if full_landing:fixture.save_landing_evidence(out)
        (out/'report.json').write_text(json.dumps(report,indent=2))
        log.close();fixture.destroy_node();rclpy.try_shutdown()
        print(out);print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
