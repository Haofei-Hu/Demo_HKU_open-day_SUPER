"""Compile the release edits against upstream trajectory timestamp containers."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/maintenance'))
import apply_super_venue_bounds as patch

class Tests(unittest.TestCase):
    def test_complete_search_release_upgrades_only_rest_contexts_and_restores(self):
        from test_demo_venue import source_tree
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source_tree(root)
            for edits in (patch.EDITS,patch.SEARCH_EDITS):
                for name,changes in edits.items():
                    p=root/name;s=p.read_text()
                    for old,new,n in changes:
                        self.assertEqual(s.count(old),n);s=s.replace(old,new)
                    p.write_text(s)
            (root/patch.HEADER).write_text(patch.HEADER_TEXT)
            before={n:(root/n).read_bytes() for n in [*patch.EDITS,patch.HEADER]}
            backup=patch.apply(root)
            record=json.loads((backup/'record.json').read_text())
            self.assertEqual(set(record['files']),{patch.ASTAR_H,patch.PLANNER})
            self.assertIsNone(patch.apply(root));patch.restore(backup)
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)

    def test_slow_rest_plan_starts_at_commit_with_backup_offset_and_shape_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'data_structure/base').mkdir(parents=True)
            (root/'data_structure/base/trajectory.h').write_text('''#pragma once
#include <vector>
#include <cstddef>
using std::size_t;
using Vec3f=double;using StatePVAJ=double;
namespace geometry_utils {
struct Polytope {void SetKnownFree(bool){}};
using PolytopeVec=std::vector<Polytope>;
struct Trajectory {
 double start_WT=0,duration=4,shape=7;
 double getTotalDuration()const{return duration;}
 double getPos(double t)const{return shape+t;}
 double getVel(double)const{return 1;}
 double getState(double t)const{return t;}
 bool getPartialTrajectoryByTime(double,double,Trajectory&)const{return true;}
};}
using geometry_utils::Polytope;
''')
            for name in ('exp_traj.h','backup_traj.h'):
                (root/name).write_bytes((ROOT/'tests/fixtures/super_venue'/('super_planner__include__data_structure__'+name+'.txt')).read_bytes())
            condition=patch.REST_EDITS[patch.PLANNER][0][1]
            code='''#include <cassert>
#include "exp_traj.h"
#include "backup_traj.h"
using namespace super_planner;
struct Astar {bool enabled;bool hasDemoVenue()const{return enabled;}};
struct Ros {double now;double getSimTime(){return now;}void info(const char*){}};
struct Record {void setBackupTraj(const Trajectory&){}void setBackupYawTraj(const Trajectory&){}
void setExpTraj(const Trajectory&){}void setExpYawTraj(const Trajectory&){} };
enum Ret {SUCCESS,FINISH,NO_NEED,FAILED};
bool rejected(bool venue,bool empty,double elapsed) {
 Astar a{venue};auto astar_ptr_=&a;ExpTraj last_exp_traj_info;Trajectory t;
 if(!empty)last_exp_traj_info.setTrajectory(0,t,t);
 struct {double replan_forward_dt=.1;} cfg_;double replan_total_t=elapsed;
'''+condition+'''return true;}return false;}
void commit(ExpTraj &exp_traj_info,BackupTraj &back_traj_info,bool venue,Ret back_ret_code,double now){
 Astar a{venue};auto astar_ptr_=&a;Ros r{now};auto ros_ptr_=&r;Record latest_replan;
'''+patch.REST_COMMIT+'''}
int main(){
 assert(!rejected(true,true,.53));assert(!rejected(true,true,20));
 assert(rejected(true,false,.53));assert(rejected(false,true,.53));
 assert(!rejected(true,false,.01));
 for(bool backup:{false,true}) for(bool venue:{false,true}) {
  ExpTraj exp;BackupTraj back;Trajectory pos,yaw;pos.duration=.4;
  exp.setTrajectory(100,pos,yaw);exp.setGoalConnectedFlag(true);exp.setWholeTrajKnownFreeFlag(true);
  exp.setSFC(geometry_utils::PolytopeVec(2));
  if(backup)back.setTrajectory(100.2,.2,pos,yaw);else back.setEmpty();
  commit(exp,back,venue,backup?SUCCESS:FINISH,103.);
  assert(exp.getStartWallTime()==(venue?103.:100.));
  assert(exp.yawTraj().start_WT==exp.getStartWallTime());
  assert(exp.getPos(0)==7 && exp.getTotalDuration()==.4);
  assert(exp.connectedToGoal() && exp.wholeTrajKnownFree() && exp.getSFCSize()==2);
  if(backup){assert(back.getStartWallTime()==(venue?103.2:100.2));
    assert(back.getStartTT()==.2 && back.getPos(0)==7);
    assert(back.yawTraj().start_WT==back.getStartWallTime());}
  else assert(back.empty());
 }
 ExpTraj exp;BackupTraj back;Trajectory t;exp.setTrajectory(100,t,t);
 commit(exp,back,true,FAILED,103);assert(exp.getStartWallTime()==100);
}
'''
            p=root/'test.cpp';p.write_text(code)
            result=subprocess.run(['g++','-std=c++17','-I',str(root),str(p),'-o',str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
