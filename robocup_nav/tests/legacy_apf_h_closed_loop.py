"""Real A*/legacy APF -> rendered H image -> projection -> alignment -> descent.

Perfect VIO/map, ideal camera extrinsics and a HYPOTHETICAL 1mm/s drift bound.
Actual calibrated K/D but synthetic 60x40cm printed H. No actuator publishers.
Not PX4 SITL, measured camera extrinsics, or permission to fly.
"""
import json
import math
import time
import cv2
import numpy as np
import yaml
from collections import deque
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
from nvblox_closed_loop import MovingFixture,ROOT,main
from cruise_band import make_band
from landing_projection import target_on_ground
from h_target_geometry import candidates
from h_target_fast import fast_candidates
from landing_sequence_shadow import LandingSequenceShadow,LandingEvidence


class HLandingFixture(MovingFixture):
    def __init__(self,controller):
        if controller not in ('legacy-apf','path-apf'):raise ValueError('This comparison requires a fixed-heading APF mode')
        super().__init__(controller)
        cv2.setNumThreads(1)
        self.pipeline.mission.band=make_band(initial_z=-1.1,rise=1.1)
        self.sequence=LandingSequenceShadow(self.pipeline,.05,allow_verified_anchor=True)
        self.ground_z=-1.24;self.floor_z=-1.1
        self.landing_state='NAVIGATE';self.goal_seen=False;self.last_image=-1.
        self.h_at=-1.;self.h_target=None;self.stable=0
        self.landing_samples=[];self.process_times=[];self.images={}
        self.states=set();self.lost_after_lock=False;self.marker=np.array([5.,2.5,self.ground_z])
        self.C=np.array([[0.,-1,0],[-1,0,0],[0,0,-1]])
        self.offset=np.array([.12,0,-.055])
        cal=yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())
        self.K=np.array(cal['camera_matrix']['data']).reshape(3,3)
        self.D=np.array(cal['distortion_coefficients']['data'])
        self.size=(int(cal['image_width']),int(cal['image_height']))
        source=cv2.imread('/home/cfly/photo_UAV/降落标志.jpg')
        if source is None:source=cv2.imread('/workspaces/ros2_ws/robocup_nav/evidence/h_reference_source.jpg')
        if source is None:raise ValueError('explicit reference image unavailable in this environment')
        detection=candidates(source)
        if len(detection)!=1:raise ValueError('reference H not unique')
        x,y,w,h=detection[0]['bbox_px'];pad=25
        self.texture=source[y-pad:y+h+pad,x-pad:x+w+pad]
        th,tw=self.texture.shape[:2]
        self.src=np.array([[0,0],[tw-1,0],[tw-1,th-1],[0,th-1]],np.float32)
        sx,sy=.6/w,.4/h  # Fixture only: NOT used by detection/projection/controller.
        self.corners=np.array([[self.marker[0]-(v-th/2)*sy,
                                self.marker[1]-(u-tw/2)*sx,self.ground_z] for u,v in self.src])
        # To synthesize raw distortion: each raw output pixel samples its
        # corresponding ideal/undistorted pixel. Original source stays untouched.
        yy,xx=np.mgrid[:self.size[1],:self.size[0]]
        raw=np.stack((xx,yy),axis=-1).astype(np.float32).reshape(-1,1,2)
        ideal=cv2.undistortPoints(raw,self.K,self.D,P=self.K).reshape(self.size[1],self.size[0],2)
        # Compile static distortion maps once; sliced float channels incurred
        # repeated conversion/copy work in every synthetic camera frame.
        self.raw_to_ideal=ideal
        self.image_worker=ThreadPoolExecutor(max_workers=1)
        self.pending_image=None

    def render(self,xy,z,yaw):
        c,s=math.cos(yaw),math.sin(yaw)
        R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        origin=np.array([*xy,z])+R@self.offset
        xyz=(self.C.T@R.T@(self.corners-origin).T).T
        uv=(self.K@xyz.T).T;uv=(uv[:,:2]/uv[:,2:]).astype(np.float32)
        H=cv2.getPerspectiveTransform(self.src,uv)
        # Fuse perspective and distortion into one resampling operation.
        texture_map=cv2.perspectiveTransform(self.raw_to_ideal,np.linalg.inv(H))
        return cv2.remap(self.texture,texture_map,None,cv2.INTER_LINEAR,borderValue=(230,230,230)),R

    def perceive(self,at,xy,z,yaw):
        image,R=self.render(xy,z,yaw);result=fast_candidates(image)
        point=None
        if len(result)==1:
            point=target_on_ground(result[0]['center_px'],self.K,self.D,self.size,self.size,
                [*xy,z],R,self.C,self.offset,self.ground_z,
                calibration_verified=True,axes_verified=True)  # Synthetic exact extrinsics ONLY.
        return at,image,point,time.monotonic()-at

    def tick(self):
        super().tick()
        if self.pending_image is not None and self.pending_image.done():
            at,image,point,elapsed=self.pending_image.result();self.pending_image=None
            self.process_times.append(elapsed)
            if point is not None:
                self.stable+=1;self.h_target=point;self.h_at=at
                if 'first_detected' not in self.images:self.images['first_detected']=image
            else:
                self.stable=0
                if self.sequence.anchor is not None:
                    self.lost_after_lock=True
                    if 'first_lost_after_lock' not in self.images:self.images['first_lost_after_lock']=image
        now=time.monotonic()
        if (self.pending_image is None and np.linalg.norm(self.xy-self.marker[:2])<=.35
                and now-self.last_image>=.12):
            self.last_image=now
            self.pending_image=self.image_worker.submit(self.perceive,now,self.xy.copy(),self.z,self.yaw)

    def destroy_node(self):
        self.image_worker.shutdown(wait=True,cancel_futures=True)
        super().destroy_node()

    def make_scene(self,now,world):
        scene=super().make_scene(now,world)
        self.goal_seen |= np.linalg.norm(self.xy-self.marker[:2])<.10
        h_valid=self.stable>=5 and 0<=now-self.h_at<=.25
        error=tuple(self.h_target[:2]-self.xy) if self.h_target is not None else (0.,0.)
        landed=self.z<=self.floor_z+.002 and self.desired_z<=self.floor_z+.002
        return replace(scene,goal_reached=self.goal_seen,h_error_odom=error,h_at=self.h_at,
                       h_metric_verified=h_valid,landing_boundary_verified=True,
                       airborne=not landed,landed=landed)

    def run_pipeline(self,now,scene,yaw_rate):
        landing=LandingEvidence(now,self.ground_z,.14,geometry_verified=True,
            column_clear=True,bounds_verified=True,armed=not scene.landed,
            anchor_drift_rate_bound_mps=.001,anchor_drift_model_verified=True)
        r=self.sequence.step(now,scene,landing,
            timestamp_us=self.get_clock().now().nanoseconds//1000,
            vio_session='synthetic',reset_counters=(0,)*5,exclusive_writer_verified=True,yaw_rate_odom=0.)
        self.landing_state=r['state'];self.states.add(r['state'])
        self.landing_samples.append(dict(at=now,state=r['state'],position=[*map(float,self.xy),self.z],
            h_valid=scene.h_metric_verified,h_error=scene.h_error_odom,reason=r.get('reason',r.get('fault','')),
            z_target=r.get('height_target_odom'),anchor_age=r.get('anchor_age_s'),
            anchor_extra_error=r.get('additional_anchor_error_m')))
        return r

    def advance_vertical(self,dt):
        desired=float(np.clip((self.desired_z-self.z)/.25,-.1,.1))
        self.vz+=float(np.clip((desired-self.vz)/.2,-.2,.2))*dt
        self.z+=self.vz*dt
        if self.z<=self.floor_z+.002 and self.desired_z<=self.floor_z+.002:
            self.z=self.floor_z;self.vz=0.

    def landing_report(self):
        return dict(landing_state=self.landing_state,landing_states=sorted(self.states),
            final_z=self.z,h_lost_after_lock=self.lost_after_lock,
            camera_frames_processed=len(self.process_times),
            processing_max_s=max(self.process_times,default=0),
            synthetic_H_size_m=[.6,.4],synthetic_cruise_rise_m=1.1,
            synthetic_drift_bound_mps=.001,actual_extrinsics_validated=False,
            real_camera_images_used=False,source_H_template='photo_UAV/降落标志.jpg',
            flight_authorized=False)

    def save_landing_evidence(self,out):
        (out/'landing_sequence.json').write_text(json.dumps(self.landing_samples,indent=2))
        for name,im in self.images.items():cv2.imwrite(str(out/(name+'.jpg')),im)


if __name__=='__main__':raise SystemExit(main(HLandingFixture,full_landing=True))
