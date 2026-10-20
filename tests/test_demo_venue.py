"""Venue/controller agreement, source rollback, and compiled search/corridor bounds."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/simulation'),str(ROOT/'scripts/flight'),
              str(ROOT/'scripts/planning'),str(ROOT/'scripts/localization'),str(ROOT/'tests')]
import apply_super_venue_bounds as source_patch
from demo_venue import halfspaces
from demo_mission import DemoMission
from super_demo_control import VenueFrame
from control_bridge_preview import quaternion_rpy
from aligned_pose_preview import rotation,multiply
from test_demo_mission import fixture
from demo_mission_simulation import rehearsal_profile

FIXTURE=ROOT/'tests/fixtures/super_venue'


def source_tree(root):
    provenance=json.loads((FIXTURE/'provenance.json').read_text())
    for name,entry in provenance['files'].items():
        path=Path(root)/name;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes((FIXTURE/entry['fixture']).read_bytes())


class Tests(unittest.TestCase):
    def geometry(self):
        world=quaternion_rpy(.25,-.17,.6);world_rotation=rotation(world)
        raw_q=quaternion_rpy(.2,-.12,.7);raw_p=np.array([.4,-.8,-3.])
        origin=np.array([1.2,-.1,.7]);translation=np.array([-.3,.5,.2]);lever=np.array([.04,-.03,.11])
        def converter(p,q,c):
            return world_rotation@(p+rotation(q)@lever)+translation,multiply(world,q),c
        ground,aligned_q,_=converter(raw_p,raw_q,np.eye(6));ground-=origin
        scene=dict(raw_position=raw_p.tolist(),raw_quaternion=raw_q.tolist(),aligned_position=ground.tolist(),
                   aligned_quaternion=aligned_q.tolist(),origin=origin.tolist())
        mission=DemoMission(DemoMission.ground_rehearsal(fixture()))
        return scene,mission,converter

    def test_rotated_tilted_frame_and_lever_match_controller_for_all_six_faces(self):
        scene,mission,convert=self.geometry();planes,record=halfspaces(scene,mission,convert)
        frame=VenueFrame(scene['aligned_position'],scene['aligned_quaternion'],mission)
        matrix=np.array(record['world_to_venue_matrix']);offset=np.array(record['world_to_venue_offset'])
        lower=np.array(record['fc_centre_lower_forward_left_floor_m'])
        upper=np.array(record['fc_centre_upper_forward_left_floor_m']);centre=(lower+upper)/2
        goal_q=np.array(record['fixed_raw_goal_quaternion_xyzw'])
        points=[centre,[0.,0.,.5],[4.5,0.,.5],[1.96,-2.23,1.66],[2.,1.,.8]]
        for i in range(3):
            for bound in (lower[i],upper[i]):
                for delta in (-.00001,0.,.00001):
                    point=centre.copy();point[i]=bound+delta;points.append(point)
        rng=np.random.default_rng(18);points.extend(rng.uniform(lower-.5,upper+.5,size=(200,3)))
        for point in points:
            raw=np.linalg.solve(matrix,np.asarray(point)-offset)
            aligned=convert(raw,goal_q,np.eye(6))[0]-scene['origin']
            np.testing.assert_allclose(frame.coordinates(aligned),point,atol=1e-12)
            admitted=bool(np.all(planes[:,:3]@raw+planes[:,3]<=1e-9))
            try:frame.check(aligned);controller=True
            except ValueError:controller=False
            self.assertEqual(admitted,controller,point)
        self.assertEqual(record['additional_boundary_inset_m'],0.)
        self.assertTrue(record['vehicle_extents_applied_once'])

    def test_frozen_scene_is_unchanged_and_physical_frame_is_not_inferred(self):
        scene,mission,convert=self.geometry();original=deepcopy(scene)
        halfspaces(scene,mission,convert);self.assertEqual(scene,original)
        with self.assertRaisesRegex(ValueError,'ground rehearsal only'):
            halfspaces(scene,DemoMission(fixture()),convert)

    def test_profile_adds_only_venue_planes_and_snapshot_io(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);source=folder/'original.yaml'
            config=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
            config['traj_opt']['boundary'].update(max_vel=2.,max_acc=1.,acceptance_max_vel=4.,acceptance_max_acc=2.)
            source.write_text(yaml.safe_dump(config));before=source.read_bytes()
            scene,mission,converter=self.geometry()
            frozen=folder/'mission.json';frozen.write_text(json.dumps(mission.config))
            with patch('demo_venue.load_converter',return_value=converter):
                scene['alignment_path']=str(folder/'alignment.py')
                result=rehearsal_profile(source,folder/'mission_report.json',scene,frozen)
            expected=deepcopy(config)
            expected['rog_map']['ros_callback']['cloud_topic']='/demo_scene/occupied_points'
            expected['rog_map']['raycasting']['enable']=False
            record=json.loads((folder/'venue_reference.json').read_text())
            expected['super_planner']['venue_halfspaces']=np.array(record['halfspaces']).reshape(-1).tolist()
            self.assertEqual(yaml.safe_load(result.read_text()),expected)
            self.assertEqual(source.read_bytes(),before)
            self.assertEqual(record['planner_profile_sha256'],hashlib.sha256(result.read_bytes()).hexdigest())
            self.assertFalse(record['captured_cloud_changed'])
            self.assertIsNone(record['search_policy']['elapsed_time_limit_s'])
            self.assertTrue(record['search_policy']['obstacle_checks_retained'])

    def test_guarded_patch_is_idempotent_and_restores_original_and_added_header(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root)
            originals={name:(root/name).read_bytes() for name in source_patch.EDITS}
            backup=source_patch.apply(root)
            self.assertEqual((root/source_patch.HEADER).read_text(),source_patch.HEADER_TEXT)
            self.assertIsNone(source_patch.apply(root))
            source_patch.restore(backup)
            self.assertFalse((root/source_patch.HEADER).exists())
            for name,data in originals.items():self.assertEqual((root/name).read_bytes(),data)

    def test_eight_marker_release_gets_map_domain_and_restores_exactly(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source_tree(root);source_patch.apply(root)
            path=root/source_patch.PLANNER
            old=path.read_text()
            for before,after,_ in source_patch.MAP_DOMAIN_EDITS[source_patch.PLANNER]:
                old=old.replace(after,before)
            path.write_text(old)
            (root/source_patch.HEADER).write_text(source_patch.HEADER_PRE_MAP_TEXT)
            backup=source_patch.apply(root)
            self.assertIn(source_patch.MAP_DOMAIN_VERSION,path.read_text())
            self.assertIsNone(source_patch.apply(root))
            source_patch.restore(backup)
            self.assertEqual(path.read_text(),old)
            self.assertEqual((root/source_patch.HEADER).read_text(),source_patch.HEADER_PRE_MAP_TEXT)
            path.write_text(old+'\n// VENUE_MAP_HEIGHT_V1 partial\n')
            with self.assertRaisesRegex(ValueError,'Partial map-height'):
                source_patch.apply(root)

    def test_unknown_or_later_modified_source_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root);bad=root/source_patch.CORRIDOR;bad.write_text('unknown')
            before={name:(root/name).read_bytes() for name in source_patch.EDITS}
            with self.assertRaisesRegex(ValueError,'Unsupported source context'):source_patch.apply(root)
            self.assertFalse((root/source_patch.HEADER).exists())
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)
            source_tree(root);backup=source_patch.apply(root)
            path=root/source_patch.PLANNER;path.write_text(path.read_text()+'\n// later edit\n')
            with self.assertRaisesRegex(ValueError,'Source changed'):source_patch.restore(backup)

    def test_nine_marker_release_upgrades_backup_variables_and_restores_exactly(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source_tree(root);source_patch.apply(root)
            path=root/source_patch.PLANNER
            old=path.read_text()
            for before,after,_ in reversed(source_patch.BACKUP_EDITS[source_patch.PLANNER]):
                old=old.replace(after,before)
            path.write_text(old)
            backup=source_patch.apply(root)
            self.assertEqual(set(json.loads((backup/'record.json').read_text())['files']),
                             {source_patch.PLANNER})
            self.assertIn(source_patch.BACKUP_VERSION,path.read_text())
            self.assertIsNone(source_patch.apply(root))
            source_patch.restore(backup)
            self.assertEqual(path.read_text(),old)
            path.write_text(old+'\n// VENUE_BACKUP_WAYPOINTS_V1 partial\n')
            with self.assertRaisesRegex(ValueError,'Partial bounded-backup'):
                source_patch.apply(root)

    def test_backup_vertex_coordinates_are_venue_only_and_preserve_motion_configuration(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            code='''#include <cassert>
#include <memory>
struct Config {int pos_constraint_type=1;double max_vel=2,max_acc=1,accept_vel=4,accept_acc=2;};
struct Logger {void info(const char*){};};
namespace traj_opt {struct BackupTrajOpt {Config used;BackupTrajOpt(Config c,Logger*):used(c){};};}
void check(bool enabled,int original) {
 struct {Config back_traj_cfg;}cfg_;cfg_.back_traj_cfg.pos_constraint_type=original;
 struct {bool value;bool enabled()const{return value;}}demo_venue{enabled};
 Logger logger;auto ros_ptr_=&logger;
 std::shared_ptr<traj_opt::BackupTrajOpt>back_traj_opt_;
'''+source_patch.BACKUP_INITIALIZATION+'''
 const auto &used=back_traj_opt_->used;
 assert(used.pos_constraint_type==(enabled?2:original));
 assert(used.max_vel==2 && used.max_acc==1 && used.accept_vel==4 && used.accept_acc==2);
 assert(cfg_.back_traj_cfg.pos_constraint_type==original);
}
int main(){check(false,1);check(false,2);check(true,1);check(true,2);}
'''
            (root/'test.cpp').write_text(code)
            result=subprocess.run(['g++','-std=c++17',str(root/'test.cpp'),'-o',str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(subprocess.run([str(root/'test')]).returncode,0)

    def test_patched_source_and_installed_binary_are_required(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'src/SUPER';source_tree(root)
            with self.assertRaisesRegex(ValueError,'Apply apply_super_venue'):source_patch.require_support(root)
            source_patch.apply(root)
            with self.assertRaisesRegex(ValueError,'rebuild'):source_patch.require_support(root)
            binary=Path(folder)/'install/super_planner/lib/super_planner/fsm_node'
            binary.parent.mkdir(parents=True);binary.write_bytes(b'\x7fELF'+source_patch.VERSION.encode())
            with self.assertRaisesRegex(ValueError,'complete bounded-search'):source_patch.require_support(root)
            binary.write_bytes(b'\x7fELF'+source_patch.VERSION.encode()+source_patch.SEARCH_VERSION.encode()+source_patch.REST_VERSION.encode()+source_patch.MAP_VERSION.encode()+source_patch.COMMIT_VERSION.encode()+source_patch.HEIGHT_VERSION.encode()+source_patch.GRID_VERSION.encode()+source_patch.START_VERSION.encode()+source_patch.MAP_DOMAIN_VERSION.encode()+source_patch.BACKUP_VERSION.encode())
            self.assertEqual(source_patch.require_support(root),binary)
            complete=binary.read_bytes()
            binary.write_bytes(complete.replace(source_patch.MAP_DOMAIN_VERSION.encode(),b''))
            with self.assertRaisesRegex(ValueError,'complete bounded-search'):
                source_patch.require_support(root)
            binary.write_bytes(complete.replace(source_patch.START_VERSION.encode(),b''))
            with self.assertRaisesRegex(ValueError,'complete bounded-search'):
                source_patch.require_support(root)
            binary.write_bytes(complete.replace(source_patch.BACKUP_VERSION.encode(),b''))
            with self.assertRaisesRegex(ValueError,'complete bounded-search'):
                source_patch.require_support(root)


    def test_previous_release_upgrades_header_map_and_commit_checks_reversibly(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root)
            for name,edits in source_patch.EDITS.items():
                if name==source_patch.INF_MAP:continue
                path=root/name;text=path.read_text()
                for group in (edits,source_patch.SEARCH_EDITS.get(name,[]),source_patch.REST_EDITS.get(name,[])):
                    for old,new,count in group:
                        self.assertEqual(text.count(old),count);text=text.replace(old,new)
                path.write_text(text)
            (root/source_patch.HEADER).write_text(source_patch.HEADER_V1_TEXT)
            before={name:(root/name).read_bytes() for name in [*source_patch.EDITS,source_patch.HEADER]}
            backup=source_patch.apply(root)
            changed=json.loads((backup/'record.json').read_text())['files']
            self.assertEqual(set(changed),{source_patch.INF_MAP,source_patch.ASTAR,source_patch.ASTAR_H,source_patch.PLANNER,source_patch.HEADER,source_patch.CORRIDOR})
            self.assertIsNone(source_patch.apply(root))
            source_patch.restore(backup)
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)

    def test_current_venue_release_upgrades_only_search_header_and_goal_handling(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root)
            for name in source_patch.EDITS:
                text=source_patch.patched(name,(root/name).read_text())
                for old,new,_ in reversed(source_patch.GRID_EDITS.get(name,[])):
                    text=text.replace(new,old)
                (root/name).write_text(text)
            (root/source_patch.HEADER).write_text(source_patch.HEADER_TEXT)
            before={name:(root/name).read_bytes() for name in [*source_patch.EDITS,source_patch.HEADER]}
            backup=source_patch.apply(root)
            self.assertEqual(set(json.loads((backup/'record.json').read_text())['files']),
                             {source_patch.ASTAR,source_patch.ASTAR_H,source_patch.PLANNER})
            self.assertIsNone(source_patch.apply(root))
            source_patch.restore(backup)
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)

    def test_search_initialization_retains_only_an_in_venue_bounded_goal(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            code='''#include <cassert>
enum Code {INIT_ERROR,SUCCESS};
bool handle(bool bounded,bool inside,Code ret_code,bool &valid) {
struct Astar {bool bounded,inside;bool hasDemoVenue()const{return bounded;}
 bool pointInsideDemoVenue(int)const{return inside;}} astar{bounded,inside};
auto astar_ptr_=&astar;int goal=0;struct {bool &goal_valid;}gi_{valid};
struct Logger {void warn(const char*){}}logger;auto ros_ptr_=&logger;
'''+source_patch.GRID_EDITS[source_patch.PLANNER][0][1]+'''
return true;
}
int main(){
bool valid=true;assert(!handle(true,true,INIT_ERROR,valid));assert(valid);
valid=true;assert(!handle(true,false,INIT_ERROR,valid));assert(!valid);
valid=true;assert(!handle(false,true,INIT_ERROR,valid));assert(!valid);
valid=true;assert(handle(true,true,SUCCESS,valid));assert(valid);
}
'''
            (root/'test.cpp').write_text(code)
            result=subprocess.run(['g++','-std=c++17',str(root/'test.cpp'),'-o',str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(subprocess.run([str(root/'test')]).returncode,0)

    def test_real_height_query_keeps_obstacle_types_and_does_not_inflate_bounds_twice(self):
        text=(FIXTURE/'rog_map__src__rog_map__inf_map.cpp.txt').read_text()
        patched=source_patch.patched(source_patch.INF_MAP,text)
        def function(source):
            start=source.index('    GridType InfMap::getGridType(const Vec3f& pos) const {')
            return source[start:source.index('\n    }',start)+6]
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);code='#include <Eigen/Dense>\n#include <cassert>\n'
            for name,src in [('before',text),('after',patched)]:
                code+='namespace '+name+r''' {
using Vec3f=Eigen::Vector3d;using Vec3i=Eigen::Vector3i;
enum GridType{OCCUPIED,UNKNOWN,KNOWN_FREE,OUT_OF_MAP};
struct InfMap {
 struct {double virtual_ceil_height=2.7,virtual_ground_height=-1.3,inflation_resolution=.2;int inflation_step=2;}cfg_;
 GridType stored=KNOWN_FREE;
 void posToGlobalIndex(const Vec3f&,Vec3i&)const{}
 GridType getGridType(const Vec3i&)const{return stored;}
 GridType getGridType(const Vec3f&)const;
};
'''+function(src)+'\n}\n'
            code+=r'''int main(){
before::InfMap old;after::InfMap fixed;
Eigen::Vector3d p(.05,.05,2.35);
assert(old.getGridType(p)==before::OCCUPIED);
assert(fixed.getGridType(p)==after::KNOWN_FREE);
p.z()=-1.1;assert(old.getGridType(p)==before::OCCUPIED);assert(fixed.getGridType(p)==after::KNOWN_FREE);
for(double z:{-1.31,-1.3,2.7,2.71}){p.z()=z;assert(fixed.getGridType(p)==after::OCCUPIED);}
p.z()=2.35;
for(auto type:{after::OCCUPIED,after::UNKNOWN,after::KNOWN_FREE,after::OUT_OF_MAP}){
 fixed.stored=type;assert(fixed.getGridType(p)==type);
}
}'''
            (folder/'test.cpp').write_text(code)
            result=subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3',str(folder/'test.cpp'),'-o',str(folder/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(subprocess.run([str(folder/'test')]).returncode,0)

    def test_continuous_candidate_check_catches_between_sample_peaks_and_outside_endpoints(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'utils/header').mkdir(parents=True)
            (root/'utils/header/yaml_loader.hpp').write_text(r'''#pragma once
#include <string>
#include <vector>
namespace yaml_loader {extern std::vector<double> values;struct YamlLoader {
 YamlLoader(const std::string&){};
 void LoadParam(const std::string&,std::vector<double>&out,const std::vector<double>&){out=values;}
};}
''')
            (root/'demo_venue.hpp').write_text(source_patch.HEADER_TEXT)
            code=r'''#include "demo_venue.hpp"
#include <cassert>
namespace yaml_loader {std::vector<double> values={1,0,0,-1,-1,0,0,0,0,1,0,-1,0,-1,0,0,0,0,1,-1,0,0,-1,0};}
struct Piece {Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,3);double duration=1;
 const Eigen::MatrixXd&getCoeffMat()const{return c;}double getDuration()const{return duration;}};
struct Trajectory{std::vector<Piece>p;int getPieceNum()const{return p.size();}const Piece&operator[](int i)const{return p[i];}};
int main(){
 super_utils::DemoVenue venue("unused"),disabled;Trajectory t;
 assert(disabled.containsTrajectory(t));assert(!venue.containsTrajectory(t));
 t.p.emplace_back();auto &p=t.p[0];p.c.col(2)<<0,.5,.5;
 p.c.row(0)<<-4,4,0;assert(venue.containsTrajectory(t)); // touches x=1 between endpoints
 p.c.row(0)<<-4.004,4.004,0;assert(!venue.containsTrajectory(t)); // 1 mm interior violation
 p.c.row(0)<<0,0,1.00098877236;assert(!venue.containsTrajectory(t)); // observed endpoint violation
 p.c.row(0)<<0,0,.5;assert(venue.containsTrajectory(t));
 p.duration=5;p.c.row(0)<<-.16,.8,0;assert(venue.containsTrajectory(t)); // normalize real-time coefficients
 p.c(0,0)=NAN;assert(!venue.containsTrajectory(t));p.c(0,0)=0;p.duration=-1;assert(!venue.containsTrajectory(t));
 // Uploaded tilted venue admits points above the initialized map ceiling.
 yaml_loader::values={0.6647647168581211,-0.7380225815829381,0.11580388721540341,-4.881888941498794,-0.6647647168581211,0.7380225815829381,-0.11580388721540341,-0.5181110585012061,0.7385968725305324,0.672552937939478,0.04633794942693744,-0.0730315487300488,-0.7385968725305324,-0.672552937939478,-0.04633794942693744,-2.326968451269951,-0.1120826976328595,0.05472855509359144,0.9921906339755021,11.98966827288536,0.1120826976328595,-0.05472855509359144,-0.9921906339755021,-14.604668272885359};
 super_utils::DemoVenue captured("unused"),bounded=captured;
 bounded.intersectMapHeight(-15.7,-11.8);
 Eigen::Vector3d q(1.9653546386302199,-3.104146540400403,-11.796818400889041);
 assert(captured.contains(q));assert(!bounded.contains(q));
 p.duration=1;p.c.setZero();p.c.col(2)=q;
 assert(captured.containsTrajectory(t));assert(!bounded.containsTrajectory(t));
 p.c(2,2)=-11.9;assert(bounded.containsTrajectory(t));
 // Endpoints inside; a mid-piece peak above the map ceiling must still fail.
 p.c.row(2)<<-.8,.8,-11.9;
 assert(captured.containsTrajectory(t));assert(!bounded.containsTrajectory(t));
 p.c.row(2)<<0,0,-11.8;assert(!bounded.containsTrajectory(t));
 Eigen::MatrixXd corridor(0,4);bounded.appendTo(corridor);assert(corridor.rows()==8);
 assert((corridor.leftCols(3)*q+corridor.col(3)).maxCoeff()>0);
 // General paired box also covers the lower face and disabled legacy behavior.
 venue.intersectMapHeight(.2,.8);
 p.c.setZero();p.c.col(2)<<.5,.5,.1;assert(!venue.containsTrajectory(t));
 p.c(2,2)=.3;assert(venue.containsTrajectory(t));
 disabled.intersectMapHeight(.2,.8);assert(disabled.contains(Eigen::Vector3d(0,0,10)));
 bool invalid=false;try{bounded.intersectMapHeight(1,0);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);

}'''
            (root/'test.cpp').write_text(code)
            result=subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3','-I'+str(root),str(root/'test.cpp'),'-o',str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(subprocess.run([str(root/'test')]).returncode,0)

    def test_rotated_boundary_grid_anchor_preserves_bounds_and_checks_connector(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'utils/header').mkdir(parents=True)
            (root/'utils/header/yaml_loader.hpp').write_text('''#pragma once
#include <string>
#include <vector>
namespace yaml_loader {extern std::vector<double> values;struct YamlLoader {
YamlLoader(const std::string&){};void LoadParam(const std::string&,std::vector<double>&v,const std::vector<double>&){v=values;}};}
''')
            (root/'demo_venue.hpp').write_text(source_patch.HEADER_TEXT)
            code=r'''#include "demo_venue.hpp"
#include <cassert>
namespace yaml_loader {std::vector<double> values;}
namespace rog_map {using Vec3f=Eigen::Vector3d;using Vec3i=Eigen::Vector3i;}
enum Type {OCCUPIED,OUT_OF_MAP,UNKNOWN,KNOWN_FREE};
struct Map {Type type=KNOWN_FREE;bool line=true,inside=true,unknown_inflation=true;mutable bool last_unknown=false;
struct Config {bool unk_inflation_en;};Config getMapConfig()const{return {unknown_inflation};}
bool insideLocalMap(const rog_map::Vec3f&)const{return inside;}
Type getInfGridType(const rog_map::Vec3f&)const{return type;}
Type getGridType(const rog_map::Vec3f&)const{return type;}
bool isLineFree(const rog_map::Vec3f&,const rog_map::Vec3f&,bool,bool unknown)const{last_unknown=unknown;return line;}};
struct Logger {template<class...Args>void info(const char*,Args&&...) {}};
struct Astar {
super_utils::DemoVenue demo_venue_;
struct {bool use_inf_map=true,use_prob_map=false,use_inf_neighbor=false,unknown_as_occ=false;}md_;
double resolution=.2;Map map;Map*map_ptr_=&map;Logger logger;Logger*ros_ptr_=&logger;bool neighbor_blocked=false;
void posToGlobalIndex(const rog_map::Vec3f&p,rog_map::Vec3i&id)const{id=(p.array()/resolution).floor().cast<int>();}
void globalIndexToPos(const rog_map::Vec3i&id,rog_map::Vec3f&p)const{p=(id.cast<double>().array()+.5)*resolution;}
bool insideLocalMap(const rog_map::Vec3i&id)const{rog_map::Vec3f p;globalIndexToPos(id,p);return demo_venue_.contains(p);}
bool neighborHaveOne(Type,const rog_map::Vec3i&){return neighbor_blocked;}
bool anchorVenueEndpoint(const rog_map::Vec3f&,rog_map::Vec3f&);
};
'''+source_patch.GRID_ANCHOR_TEXT+r'''
int main(){
Eigen::Matrix3d m;m<<.7062656002273738,-.7049376218970616,.06520622032891805,
.7069247418983289,.7071958205848822,-.011466500741575328,
-.03803039872850083,.05419428550494229,.9978059271176735;
Eigen::Vector3d offset(-.8204280173519238,-3.593843227716729,.444714171600531);
Eigen::Vector3d lower(-.3,-1.2,.085),upper(5.1,1.2,2.7);
for(int i=0;i<3;++i)for(int sign:{1,-1}){
 for(int j=0;j<3;++j)yaml_loader::values.push_back(sign*m(i,j));
 yaml_loader::values.push_back(sign==1?offset[i]-upper[i]:lower[i]-offset[i]);}
Astar a;a.demo_venue_=super_utils::DemoVenue("unused");
const Eigen::Vector3d real(5.297986668861563,.1612586833881705,2.448742985653654);
Eigen::Vector3d anchor,snapped;Eigen::Vector3i id;a.posToGlobalIndex(real,id);a.globalIndexToPos(id,snapped);
assert(a.demo_venue_.contains(real));assert(!a.demo_venue_.contains(snapped)); // observed regression
assert(a.anchorVenueEndpoint(real,anchor));assert(a.demo_venue_.contains(anchor));assert((anchor-real).norm()<.4);
a.map.line=false;assert(!a.anchorVenueEndpoint(real,anchor));a.map.line=true;
a.map.type=OCCUPIED;assert(!a.anchorVenueEndpoint(real,anchor));
a.map.type=OUT_OF_MAP;assert(!a.anchorVenueEndpoint(real,anchor));
a.map.type=UNKNOWN;a.md_.unknown_as_occ=true;assert(!a.anchorVenueEndpoint(real,anchor));
a.md_.unknown_as_occ=false;assert(a.anchorVenueEndpoint(real,anchor));a.map.type=KNOWN_FREE;
a.md_.unknown_as_occ=true;assert(a.anchorVenueEndpoint(real,anchor));assert(a.map.last_unknown);
a.map.unknown_inflation=false;assert(a.anchorVenueEndpoint(real,anchor));assert(!a.map.last_unknown);
a.md_.unknown_as_occ=false;
a.map.inside=false;assert(!a.anchorVenueEndpoint(real,anchor));a.map.inside=true;
a.md_.use_prob_map=true;a.md_.use_inf_neighbor=true;a.neighbor_blocked=true;
assert(!a.anchorVenueEndpoint(real,anchor));a.neighbor_blocked=false;
int recovered=0;
for(int face=0;face<3;++face)for(double bound:{lower[face]+.001,upper[face]-.001}){
 Eigen::Vector3d v=(lower+upper)/2;v[face]=bound;Eigen::Vector3d p=m.transpose()*(v-offset);
 a.posToGlobalIndex(p,id);a.globalIndexToPos(id,snapped);
 assert(a.anchorVenueEndpoint(p,anchor));assert(a.demo_venue_.contains(anchor));
 if(!a.demo_venue_.contains(snapped))++recovered;
}
assert(recovered>=3);
Eigen::Vector3d invalid=m.transpose()*(upper+Eigen::Vector3d::Ones()*.001-offset);
assert(!a.anchorVenueEndpoint(invalid,anchor));
a.demo_venue_=super_utils::DemoVenue();assert(a.anchorVenueEndpoint(invalid,anchor));assert(anchor==invalid);
}'''
            (root/'test.cpp').write_text(code)
            result=subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3','-I'+str(root),str(root/'test.cpp'),'-o',str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(root/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_deployed_venue_v1_upgrade_changes_only_search_and_planner_startup(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root)
            for name,edits in source_patch.EDITS.items():
                path=root/name;text=path.read_text()
                for old,new,count in edits:
                    self.assertEqual(text.count(old),count);text=text.replace(old,new)
                path.write_text(text)
            (root/source_patch.HEADER).write_text(source_patch.HEADER_TEXT)
            before={name:(root/name).read_bytes() for name in [*source_patch.EDITS,source_patch.HEADER]}
            backup=source_patch.apply(root)
            changed=json.loads((backup/'record.json').read_text())['files']
            self.assertEqual(set(changed),{source_patch.ASTAR,source_patch.ASTAR_H,source_patch.PLANNER,source_patch.CORRIDOR})
            self.assertEqual((root/source_patch.HEADER).read_bytes(),before[source_patch.HEADER])
            self.assertIsNone(source_patch.apply(root))
            source_patch.restore(backup)
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)

    def test_search_budget_guards_compile_and_preserve_profiles_without_venue(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            code='''#include <cassert>
struct Venue {bool active;bool enabled() const{return active;}};
'''
            for index,(old,new,_) in enumerate(source_patch.SEARCH_EDITS[source_patch.ASTAR][:3]):
                for label,condition in (('old',old),('new',new)):
                    code+=f'''bool {label}{index}(bool active,double elapsed,bool visual=false) {{
Venue demo_venue_{{active}};struct {{bool visual_process;}} cfg_{{visual}};
double time_1=0,time_2=elapsed,time_out=.1;
{condition} return true;
return false;
}}
'''
            code+='int main() {\n'
            for index in range(3):
                code+=f'''assert(old{index}(true,.35));
assert(!new{index}(true,.35));
assert(new{index}(false,.35));
assert(!new{index}(false,.01));
'''
            code+='}\n'
            path=root/'guard.cpp';path.write_text(code)
            result=subprocess.run(['g++','-std=c++17',str(path),'-o',str(root/'guard')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(root/'guard')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_compiled_search_predicate_and_corridor_intersection(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'utils/header').mkdir(parents=True)
            (root/'utils/header/yaml_loader.hpp').write_text('''#pragma once
#include <string>
#include <vector>
namespace yaml_loader {extern std::vector<double> values; struct YamlLoader {
explicit YamlLoader(const std::string&) {}
void LoadParam(const std::string&,std::vector<double>& out,const std::vector<double>&) {out=values;}
};}
''')
            (root/'demo_venue.hpp').write_text(source_patch.HEADER_TEXT)
            original=(FIXTURE/'astar_predicate.cpp').read_text()
            before,after,count=source_patch.EDITS[source_patch.ASTAR][0]
            self.assertEqual(original.count(before),count);predicate=original.replace(before,after)
            code='''#include <cassert>
#include <queue>
#include <set>
#include "demo_venue.hpp"
std::vector<double> yaml_loader::values;
namespace rog_map {using Vec3i=Eigen::Vector3i;using Vec3f=Eigen::Vector3d;}
struct Astar {
 super_utils::DemoVenue demo_venue_;
 struct {rog_map::Vec3i local_map_center_id_g{0,0,0};} md_;
 struct {rog_map::Vec3i map_size_i{100,100,100};} cfg_;
 void globalIndexToPos(const rog_map::Vec3i& id,rog_map::Vec3f& p) const {p=id.cast<double>()*.1;}
 bool insideLocalMap(const rog_map::Vec3i&) const;
};
'''+predicate+'''
int main() {
 super_utils::DemoVenue absent("unused");assert(absent.contains({1000,1000,1000}));
 yaml_loader::values={1,0,0,-5.1, -1,0,0,-.3, 0,1,0,-1.2, 0,-1,0,-1.2, 0,0,1,-2.7, 0,0,-1,.085};
 super_utils::DemoVenue venue("unused");
 assert(venue.contains({0,0,.5}));assert(venue.contains({4.5,0,.5}));
 assert(!venue.contains({1.9569546619,-2.2295969360,1.6620323888}));
 assert(venue.contains({2,1,.8}));assert(venue.contains({2,1.2,.8}));
 assert(!venue.contains({2,1.20001,.8}));
 Astar search;search.demo_venue_=venue;
 assert(search.insideLocalMap({0,0,5}));assert(!search.insideLocalMap({20,-23,16}));
 // An obstacle blocks the straight route. Search must curve inside the same box.
 std::queue<std::pair<int,int>> pending;std::set<std::pair<int,int>> seen;
 pending.push({0,0});seen.insert({0,0});bool reached=false,curved=false;
 while(!pending.empty()) {auto p=pending.front();pending.pop();
   if(p==std::make_pair(45,0)) {reached=true;break;}
   for(auto d:std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}) {
     auto q=std::make_pair(p.first+d.first,p.second+d.second);
     if(!search.insideLocalMap({q.first,q.second,5}) || seen.count(q)) continue;
     if(q.first>=19 && q.first<=21 && std::abs(q.second)<=5) continue;
     if(std::abs(q.second)>5) curved=true;
     seen.insert(q);pending.push(q);
   }
 }
 assert(reached && curved);
 Eigen::Matrix<double,Eigen::Dynamic,4> corridor(6,4);
 corridor<<1,0,0,-6, -1,0,0,-1, 0,1,0,-3, 0,-1,0,-3, 0,0,1,-3, 0,0,-1,0;
 venue.appendTo(corridor);assert(corridor.rows()==12);
 Eigen::Vector3d rejected(1.9569546619,-2.2295969360,1.6620323888),allowed(2,1,.8);
 assert((corridor.leftCols<3>()*rejected+corridor.col(3)).maxCoeff()>1.);
 assert((corridor.leftCols<3>()*allowed+corridor.col(3)).maxCoeff()<=1e-9);
 yaml_loader::values[7]=5.2;bool invalid=false;
 try {super_utils::DemoVenue bad("unused");}catch(const std::invalid_argument&) {invalid=true;}
 assert(invalid);
}
'''
            source=root/'check.cpp';source.write_text(code)
            result=subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3','-I'+str(root),str(source),'-o',str(root/'check')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(root/'check')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_compiled_actual_corridor_generators_clip_all_five_success_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'utils/header').mkdir(parents=True)
            (root/'utils/header/yaml_loader.hpp').write_text('''#pragma once
#include <string>
#include <vector>
namespace yaml_loader {extern std::vector<double> values; struct YamlLoader {
explicit YamlLoader(const std::string&) {}
void LoadParam(const std::string&,std::vector<double>& out,const std::vector<double>&) {out=values;}
};}
''')
            (root/'demo_venue.hpp').write_text(source_patch.HEADER_TEXT)
            original=(FIXTURE/'corridor_generators.cpp').read_text()
            functions=original
            for before,after,count in source_patch.EDITS[source_patch.CORRIDOR]+source_patch.HEIGHT_EDITS[source_patch.CORRIDOR]:
                if before in functions:
                    self.assertEqual(functions.count(before),count)
                    functions=functions.replace(before,after)
            code='''#include <cassert>
#include <memory>
#include <utility>
#include "demo_venue.hpp"
std::vector<double> yaml_loader::values;
using Vec3f=Eigen::Vector3d;using Vec3i=Eigen::Vector3i;
namespace super_utils {using Vec3f=Eigen::Vector3d;}
using MatD4f=Eigen::Matrix<double,Eigen::Dynamic,4>;
template<class T> using vec_E=std::vector<T,Eigen::aligned_allocator<T>>;
using Line=std::pair<Vec3f,Vec3f>;using namespace std;
const char* YELLOW="";const char* RESET="";const int OCCUPIED=1;
enum RET_CODE {SUCCESS,FAILED};
struct Polytope {
 MatD4f planes;
 MatD4f GetPlanes() const {return planes;}
 void SetPlanes(const MatD4f& p) {planes=p;}
 void SetSeedLine(const Line&) {}
 void Reset() {planes.resize(0,4);}
};
namespace rog_map {struct TimeConsuming {TimeConsuming(const char*,bool){} double stop(){return 0;}};}
struct FakeMap {
 bool with_obstacle=false;
 void boundBoxByLocalMap(Vec3f&low,Vec3f&high) {low.z()=std::max(low.z(),-1.3);high.z()=std::min(high.z(),2.7);}
 void boxSearch(const Vec3f&,const Vec3f&,int,vec_E<Vec3f>& pc) {if(with_obstacle)pc.push_back({5,5,5});}
};
struct FakeCiri {
 MatD4f bd;
 template<class M> RET_CODE comvexDecomposition(const MatD4f& b,const M&,const Vec3f&,const Vec3f&) {bd=b;return SUCCESS;}
 void getPolytope(Polytope& p) {p.SetPlanes(bd);}
};
namespace super_planner {
class CorridorGenerator {
public:
 double bound_dis_=1.,robot_r_=.2,virtual_groud_height_=-1.3,virtual_ceil_height_=2.7;
 shared_ptr<FakeMap> map_ptr_=make_shared<FakeMap>();
 shared_ptr<FakeCiri> ciri_=make_shared<FakeCiri>();
 vec_E<Vec3f> latest_pc;int ciri_cnt=0;double ciri_t=0;
'''+source_patch.CLIP+'''
 void getSeedBBox(const Vec3f&,const Vec3f&,Vec3f&,Vec3f&);
 bool GeneratePolytopeFromPoint(const Vec3f&,Polytope&);
 bool GenerateEmptyPolytope(const Vec3f&,const double&,Polytope&);
 bool GeneratePolytopeFromLine(Line&,Polytope&);
};
'''+functions+'''
int main() {
 super_planner::CorridorGenerator generator;Polytope p;Line line{{2,1.1,.5},{3,1.1,.5}};
 assert(generator.GeneratePolytopeFromLine(line,p));assert(p.GetPlanes().rows()==6);
 yaml_loader::values={1,0,0,-5.1, -1,0,0,-.3, 0,1,0,-1.2, 0,-1,0,-1.2, 0,0,1,-2.7, 0,0,-1,.085};
 generator.demo_venue_=super_utils::DemoVenue("unused");
 auto checked=[&]() {
   auto planes=p.GetPlanes();assert(planes.rows()==12);
   Vec3f inside(2.5,1.1,.5),outside(2.5,1.21,.5);
   assert((planes.leftCols<3>()*inside+planes.col(3)).maxCoeff()<=1e-9);
   assert((planes.leftCols<3>()*outside+planes.col(3)).maxCoeff()>0);
 };
 // Empty-cloud point and line, both CIRI success branches, and fallback box.
 for(bool obstacle:{false,true}) {
   generator.map_ptr_->with_obstacle=obstacle;
   assert(generator.GeneratePolytopeFromPoint({2.5,1.1,.5},p));checked();
   assert(generator.GeneratePolytopeFromLine(line,p));checked();
 }
 assert(generator.GenerateEmptyPolytope({2.5,1.1,.5},1.,p));checked();
 // Already-inflated virtual ceiling admits the observed 2.55 m seed.
 assert(generator.GeneratePolytopeFromPoint({2.5,1.1,2.55},p));
 assert((p.GetPlanes().leftCols<3>()*Vec3f(2.5,1.1,2.55)+p.GetPlanes().col(3)).maxCoeff()<=1e-9);
 assert(std::abs(p.GetPlanes()(4,3)+2.7)<1e-9);
 assert(generator.GeneratePolytopeFromPoint({2.5,1.1,.5},p));
 assert(std::abs(p.GetPlanes()(4,3)+1.3)<1e-9); // ordinary crop still inset by radius
 Vec3f bad(1.9569546619,-2.2295969360,1.6620323888);
 assert(!generator.GeneratePolytopeFromPoint(bad,p));
 assert(!generator.GenerateEmptyPolytope(bad,1.,p));
 line.second=bad;assert(!generator.GeneratePolytopeFromLine(line,p));
}
'''
            source=root/'check.cpp';source.write_text(code)
            result=subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3','-I'+str(root),str(source),'-o',str(root/'check')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(root/'check')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
