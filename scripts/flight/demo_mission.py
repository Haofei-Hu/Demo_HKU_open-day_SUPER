#!/usr/bin/env python3
"""Canonical demonstration geometry and report; no ROS or aircraft operations."""
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path


class DemoMission:
    MOTION_LIMITS=(2.,1.)
    PROFILES={'open_day_out_and_back':(4.5,5.8,'SUPER_OUTBOUND_4_5M',True),
              'short_site_1m_rehearsal':(1.,2.,'SUPER_OUTBOUND_1M',False)}
    PHASES=('TAKEOFF_TO_1_5M','STABILIZE_AT_HOME','SUPER_OUTBOUND_4_5M',
            'STABILIZE_AT_FAR_POINT','SUPER_RETURN_HOME','HOLD_HOME_6S',
            'AUTO_LAND','CONFIRM_LANDED_AND_DISARMED')

    def __init__(self,config):
        self.config=deepcopy(config)
        if config.get('schema_version')!=1:raise ValueError('Unsupported mission schema')
        profile=config.get('mission_id')
        if profile not in self.PROFILES:
            raise ValueError('Unknown mission profile')
        self.distance,venue_length,outbound,self.demonstration=self.PROFILES[profile]
        context=config.get('execution_context')
        if context not in (None,'ground_rehearsal'):raise ValueError('Unknown mission execution context')
        self.rehearsal_only=context=='ground_rehearsal'
        v=config['venue'];f=config['flight'];s=config['takeoff'];body=config['vehicle']
        self.round_trips=f.get('round_trips',1)
        if isinstance(self.round_trips,bool) or not isinstance(self.round_trips,int) or self.round_trips<1:
            raise ValueError('Round trips must be a positive integer')
        self.outbound_stage=outbound
        height=f.get('height_fc_above_floor_m')
        allowed_heights=(.5,1.5) if self.rehearsal_only else (1.,1.5,1.8)
        if isinstance(height,bool) or height not in allowed_heights:
            raise ValueError('Final demonstration settings differ: unsupported height')
        takeoff_stage='TAKEOFF_TO_'+str(height).replace('.','_')+'M'
        stages=[takeoff_stage,'STABILIZE_AT_HOME']
        for number in range(1,self.round_trips+1):
            stages.extend(self.round_stage(stage,number) for stage in
                          (outbound,'STABILIZE_AT_FAR_POINT','SUPER_RETURN_HOME'))
        self.PHASES=tuple(stages+['HOLD_HOME_6S','AUTO_LAND','CONFIRM_LANDED_AND_DISARMED'])
        if v.get('boundary_type')!='outer':raise ValueError('Mission requires surveyed outer boundaries')
        calibrated=v.get('calibration_method') is not None or 'venue_calibration' in config
        if calibrated:
            from rviz_venue_calibration import validate_geometry,REHEARSAL_SCOPE
            calibration=validate_geometry(config)
            if self.rehearsal_only != (calibration.get('scope')==REHEARSAL_SCOPE):
                raise ValueError('Venue calibration execution scope does not match the mission')
        fixed={'length_forward_m':venue_length,'height_m':2.8}
        if calibrated:fixed['height_m']=calibration['geometry']['floor_height_limits_m'][1]
        if not calibrated:fixed['width_left_m']=2.8
        if any(v.get(k)!=n for k,n in fixed.items()):raise ValueError('Venue differs from the selected mission profile')
        expected={'height_fc_above_floor_m':height,'outbound_forward_m':self.distance,
                  'horizontal_speed_limit_mps':self.MOTION_LIMITS[0],'acceleration_limit_mps2':self.MOTION_LIMITS[1],
                  'return_hover_s':6.,'axis_error_limit_m':.1}
        if any(f.get(k)!=n for k,n in expected.items()):raise ValueError('Final demonstration settings differ')
        if f.get('yaw_policy')!='hold_takeoff_heading':raise ValueError('Need fixed takeoff heading')
        if s.get('heading_along_length_confirmed') is not True:raise ValueError('Confirm heading along venue length')
        self.hover_height=float(f['height_fc_above_floor_m'])
        self.hull={name:self.measure(body.get('centre_to_'+name+'_m'),'FC-centre to '+name)
                   for name in ('front','rear','left','right','top','bottom')}
        self.rear=self.measure(s.get('forward_from_rear_m'),'takeoff distance from rear boundary')
        self.left=self.measure(s.get('distance_from_left_m'),'takeoff distance from left boundary')
        self.width=float(v['width_left_m']);self.length=fixed['length_forward_m'];self.height=float(fixed['height_m']);self.tolerance=.1
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

    def round_stage(self,stage,number):
        return stage if self.round_trips==1 else f'ROUND_{number:02d}_{stage}'

    @classmethod
    def load(cls,path):
        return cls(json.loads(Path(path).read_text()))

    @classmethod
    def physical_height(cls,config,height):
        """Create a validated flight copy; retain the surveyed geometry and route."""
        if config.get('execution_context') is not None or 'SYNTHETIC' in str(config.get('status','')).upper():
            raise ValueError('Physical height override requires a measured physical mission')
        result=deepcopy(config)
        result['flight']['height_fc_above_floor_m']=height
        mission=cls(result)
        result['sequence']=list(mission.PHASES)
        return result

    @staticmethod
    def ground_rehearsal(config,height=None):
        """Use a per-run virtual height; preserve it through later preparation stages."""
        result=deepcopy(config)
        if height is None:
            height=config['flight']['height_fc_above_floor_m'] if config.get('execution_context')=='ground_rehearsal' else .5
        if isinstance(height,bool) or height not in (.5,1.5):
            raise ValueError('Unsupported rehearsal height')
        result['execution_context']='ground_rehearsal'
        result['flight']['height_fc_above_floor_m']=height
        result['sequence'][0]='TAKEOFF_TO_'+str(height).replace('.','_')+'M'
        if 'venue_calibration' in result:
            from rviz_venue_calibration import REHEARSAL_SCOPE
            result['venue_calibration']['scope']=REHEARSAL_SCOPE
        return result

    @staticmethod
    def short_site_fixture(config):
        """Separate virtual fixture; never overwrite the canonical venue survey."""
        result=deepcopy(config)
        result['mission_id']='short_site_1m_rehearsal'
        result['venue'].update(length_forward_m=2.,boundary_type='outer')
        result['flight']['outbound_forward_m']=1.
        result['sequence']=[p.replace('SUPER_OUTBOUND_4_5M','SUPER_OUTBOUND_1M') for p in result['sequence']]
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
            raise ValueError('Ground rehearsal height is for virtual execution only; use a measured physical mission')
        self.path=Path(path)
        self.phases=mission.PHASES
        self.demonstration=mission.demonstration
        self.round_returns={mission.round_stage('SUPER_RETURN_HOME',number) for number in range(1,mission.round_trips+1)}
        if self.path.exists():raise ValueError('Mission report already exists; use a new path')
        self.data=dict(schema_version=1,result='RUNNING',mode=mode,
            execution_context='ground_rehearsal' if mission.rehearsal_only else 'physical_mission_specification',
            requested_fc_floor_height_m=mission.hover_height,
            round_trips_requested=mission.round_trips,round_trips_completed=0,
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
        self.data['completed_stages'].append(stage)
        if stage in self.round_returns:self.data['round_trips_completed']+=1
        self.save()

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
