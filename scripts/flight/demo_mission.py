#!/usr/bin/env python3
"""Canonical demonstration geometry and report; no ROS or aircraft operations."""
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path


class DemoMission:
    PHASES=('TAKEOFF_TO_1_5M','STABILIZE_AT_HOME','SUPER_OUTBOUND_4_5M',
            'STABILIZE_AT_FAR_POINT','SUPER_RETURN_HOME','HOLD_HOME_6S',
            'AUTO_LAND','CONFIRM_LANDED_AND_DISARMED')

    def __init__(self,config):
        self.config=deepcopy(config)
        if config.get('schema_version')!=1:raise ValueError('Unsupported mission schema')
        profile=config.get('mission_id')
        if profile not in ('open_day_out_and_back','short_site_1m_rehearsal'):
            raise ValueError('Unknown mission profile')
        self.demonstration=profile=='open_day_out_and_back'
        context=config.get('execution_context')
        if context not in (None,'ground_rehearsal'):raise ValueError('Unknown mission execution context')
        self.rehearsal_only=context=='ground_rehearsal'
        self.distance=4.5 if self.demonstration else 1.
        self.PHASES=tuple('TAKEOFF_TO_0_5M' if self.rehearsal_only and p=='TAKEOFF_TO_1_5M' else
                          'SUPER_OUTBOUND_1M' if not self.demonstration and p=='SUPER_OUTBOUND_4_5M'
                          else p for p in type(self).PHASES)
        v=config['venue'];f=config['flight'];s=config['takeoff'];body=config['vehicle']
        if v.get('boundary_type')!='outer':raise ValueError('Mission requires surveyed outer boundaries')
        fixed={'length_forward_m':5.8 if self.demonstration else 2.,'width_left_m':2.8,'height_m':2.8}
        if any(v.get(k)!=n for k,n in fixed.items()):raise ValueError('Venue differs from the selected mission profile')
        expected={'height_fc_above_floor_m':.5 if self.rehearsal_only else 1.5,'outbound_forward_m':self.distance,
                  'horizontal_speed_limit_mps':2.,'acceleration_limit_mps2':1.,
                  'return_hover_s':6.,'axis_error_limit_m':.1}
        if any(f.get(k)!=n for k,n in expected.items()):raise ValueError('Final demonstration settings differ')
        if f.get('yaw_policy')!='hold_takeoff_heading':raise ValueError('Need fixed takeoff heading')
        if s.get('heading_along_length_confirmed') is not True:raise ValueError('Confirm heading along venue length')
        self.hover_height=float(f['height_fc_above_floor_m'])
        self.hull={name:self.measure(body.get('centre_to_'+name+'_m'),'FC-centre to '+name)
                   for name in ('front','rear','left','right','top','bottom')}
        self.rear=self.measure(s.get('forward_from_rear_m'),'takeoff distance from rear boundary')
        self.left=self.measure(s.get('distance_from_left_m'),'takeoff distance from left boundary')
        self.width=2.8;self.length=fixed['length_forward_m'];self.height=2.8;self.tolerance=.1
        if not 0<self.hull['bottom']<=.085:
            raise ValueError('Measured FC-to-bottom clearance must fit the existing 0.085 m floor calibration')
        # Validate the requested route's hull against the outer venue directly.
        # The arrival tolerance is not an additional corridor or boundary inset.
        for forward in (0.,self.distance):self.check((forward,0.,self.hover_height))
        self.check((0.,0.,.085),on_floor=True)

    @staticmethod
    def measure(value,name):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
            raise ValueError('Missing/invalid measured geometry: '+name)
        return float(value)

    @classmethod
    def load(cls,path):
        return cls(json.loads(Path(path).read_text()))

    @staticmethod
    def ground_rehearsal(config):
        """Use a per-run 0.5 m virtual route; never edit the 1.5 m flight file."""
        result=deepcopy(config)
        result['execution_context']='ground_rehearsal'
        result['flight']['height_fc_above_floor_m']=.5
        result['sequence'][0]='TAKEOFF_TO_0_5M'
        return result

    @staticmethod
    def short_site_fixture(config):
        """Separate virtual fixture; never overwrite the canonical venue survey."""
        result=deepcopy(config)
        result['mission_id']='short_site_1m_rehearsal'
        result['venue'].update(length_forward_m=2.,boundary_type='outer')
        result['flight']['outbound_forward_m']=1.
        result['sequence']=[p if p!='SUPER_OUTBOUND_4_5M' else 'SUPER_OUTBOUND_1M'
                            for p in result['sequence']]
        result['status']='SYNTHETIC_SHORT_SITE_REHEARSAL_NOT_A_FLIGHT_CONFIGURATION'
        result['local_test_context']=dict(forward_obstacle_distance_approx_m=1.5,
            venue_geometry='Virtual test box; current room walls and aircraft extents are not surveyed',
            final_demonstration_distance_m=4.5,demonstration_completion=False)
        return result

    def check(self,point,on_floor=False):
        if len(point)!=3 or not all(math.isfinite(v) for v in point):raise ValueError('Invalid venue point')
        x,y,z=self.rear+point[0],self.width-self.left+point[1],point[2]
        h=self.hull
        clearances={'rear':x-h['rear'],'front':self.length-x-h['front'],
                    'right':y-h['right'],'left':self.width-y-h['left'],
                    'ceiling':self.height-z-h['top']}
        if not on_floor:clearances['floor']=z-h['bottom']
        if any(value < -1e-9 for value in clearances.values()):
            raise ValueError(f'Aircraft envelope outside venue: relative FC xyz={list(point)}; clearance m={clearances}')
        return clearances

    def waypoints(self):
        return {'ground':[0.,0.,.085],'home_hover':[0.,0.,self.hover_height],
                'far_hover':[self.distance,0.,self.hover_height],'return_hover':[0.,0.,self.hover_height]}


class MissionReport:
    def __init__(self,path,mission,mode):
        if mission.rehearsal_only and mode!='simulation':
            raise ValueError('Ground rehearsal height is for virtual execution only; physical tests require 1.5 m')
        self.path=Path(path)
        self.phases=mission.PHASES
        self.demonstration=mission.demonstration
        if self.path.exists():raise ValueError('Mission report already exists; use a new path')
        self.data=dict(schema_version=1,result='RUNNING',mode=mode,
            execution_context='ground_rehearsal' if mission.rehearsal_only else 'physical_mission_specification',
            requested_fc_floor_height_m=mission.hover_height,
            mission=mission.config,waypoints=mission.waypoints(),expected_stages=list(self.phases),events=[],
            completed_stages=[],error=None,physical_flight_verified=False,
            started_at=datetime.now().astimezone().isoformat())
        self.save()

    def save(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.data,indent=2,allow_nan=False)+'\n');temp.replace(self.path)

    def event(self,phase,**details):
        self.data['events'].append(dict(phase=phase,time=datetime.now().astimezone().isoformat(),**details))
        self.data['stage']=phase;self.save()

    def completed(self,stage):
        if stage in self.data['completed_stages']:raise ValueError('Mission stage repeated: '+stage)
        expected=self.phases[len(self.data['completed_stages'])]
        if stage!=expected:raise ValueError('Mission stage out of order: '+stage+'; expected '+expected)
        self.data['completed_stages'].append(stage);self.save()

    def finish(self,error=None):
        complete=self.data['completed_stages']==list(self.phases)
        self.data.update(result='SUCCESS' if complete and error is None else 'FAIL',error=error,
            physical_flight_verified=bool(complete and error is None and self.data['mode']=='flight' and self.demonstration),
            demonstration_completed=bool(complete and error is None and self.demonstration),
            finished_at=datetime.now().astimezone().isoformat())
        self.save()
        print('RESULT:',self.data['result'],flush=True);print('REPORT:',self.path,flush=True)


def default_report_path():
    return Path.home()/'Downloads/super_demo_reports'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')/'report.json'
