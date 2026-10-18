"""Compile the real PlanFromRest start-selection edit with recorded coordinates."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/maintenance'))
import apply_super_venue_bounds as patch


class Tests(unittest.TestCase):
    def test_recorded_snap_is_removed_without_changing_free_space_or_legacy_policy(self):
        # Compile both exact upstream and replacement blocks, including their
        # shared rejection/recording code, rather than reimplementing the branch.
        fixture=ROOT/'tests/fixtures/super_venue/super_planner__src__super_core__super_planner.cpp.txt'
        source=fixture.read_text()
        start=source.index(patch.REST_START_OLD)
        end=source.index('        /// 2) Generate Exp traj',start)
        original=source[start:end]
        updated=original.replace(patch.REST_START_OLD,patch.REST_START_NEW)
        preamble=r'''
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <limits>
using Vec3f=Eigen::Vector3d;
enum class GridType {KNOWN_FREE,UNKNOWN,OCCUPIED,OUT_OF_MAP};
enum class SUPER_RET_CODE {SUPER_NO_START_POINT};
const bool FAILED=false;
struct Map {
 GridType type=GridType::KNOWN_FREE;bool inside=true,nearest_ok=true;int nearest_calls=0,queries=0;
 Vec3f snapped{4.15,-3.45,1.65};
 bool insideLocalMap(const Vec3f&)const{return inside;}
 GridType getGridType(const Vec3f&){++queries;return type;}
 bool getNearestCellNot(GridType,const Vec3f&,Vec3f &out,double){++nearest_calls;out=snapped;return nearest_ok;}
};
struct Astar {bool venue=true,contains=true;bool hasDemoVenue()const{return venue;}
 bool pointInsideDemoVenue(const Vec3f&)const{return contains;}};
struct Ros {template<class... T>void info(T&&...){}template<class... T>void warn(T&&...){}
 template<class... T>void error(T&&...){} };
struct Record {Vec3f result{999,999,999};bool rejected=false;
 void setRetCode(SUPER_RET_CODE){rejected=true;}void setLocalStartP(const Vec3f&p){result=p;}};
struct Robot {Vec3f p;};
'''
        def function(name,block):
            return 'bool '+name+'(Map *map_ptr_,Astar *astar_ptr_,Robot robot_state_,Record &latest_replan){Ros r;auto ros_ptr_=&r;\n'+block+'return true;}\n'
        code=preamble+function('before',original)+function('after',updated)+r'''
int main(){
 const Vec3f stop{4.1737,-3.4012,1.6464};Robot robot{stop};
 Map map;Astar astar;Record old_record,new_record;
 assert(before(&map,&astar,robot,old_record));
 assert(std::abs((old_record.result-stop).norm()-.05436993654585225)<1e-12);
 assert(after(&map,&astar,robot,new_record));
 assert((new_record.result-stop).norm()==0.0);assert(map.nearest_calls==1);
 for(auto type:{GridType::KNOWN_FREE,GridType::UNKNOWN}) {
  Map m;m.type=type;Record r;assert(after(&m,&astar,robot,r));
  assert(m.nearest_calls==0);assert((r.result-stop).norm()==0.0);
 }
 for(auto type:{GridType::OCCUPIED,GridType::OUT_OF_MAP}) {
  Map m;m.type=type;Record r;assert(!after(&m,&astar,robot,r));
  assert(r.rejected);assert(m.nearest_calls==0);assert(r.result.x()==999);
 }
 for(int bad=0;bad<3;++bad) {
  Map m;Astar a;Robot rob{stop};Record r;
  if(bad==0)m.inside=false;
  if(bad==1)a.contains=false;
  if(bad==2)rob.p.x()=std::numeric_limits<double>::quiet_NaN();
  assert(!after(&m,&a,rob,r));assert(r.rejected);assert(m.nearest_calls==0);assert(m.queries==0);
 }
 // Without venue constraints, retain the exact upstream relocation behavior.
 Astar legacy;legacy.venue=false;Map m;Record r;
 assert(after(&m,&legacy,robot,r));assert(m.nearest_calls==1);assert(r.result==m.snapped);
 m.nearest_ok=false;Record fail;assert(!after(&m,&legacy,robot,fail));assert(fail.rejected);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);cpp=folder/'test.cpp';cpp.write_text(code)
            subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3',str(cpp),'-o',str(folder/'test')],check=True)
            subprocess.run([str(folder/'test')],check=True)

    def test_old_complete_release_upgrades_and_restores_without_partial_writes(self):
        from test_demo_venue import source_tree
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source_tree(root);patch.apply(root)
            path=root/patch.PLANNER
            previous=path.read_text().replace(patch.REST_START_NEW,patch.REST_START_OLD)
            path.write_text(previous)
            backup=patch.apply(root)
            self.assertIn(patch.START_VERSION,path.read_text())
            self.assertIsNone(patch.apply(root))
            patch.restore(backup);self.assertEqual(path.read_text(),previous)
            path.write_text(previous.replace('Vec3f local_star_pt;','Vec3f changed_local_start;'))
            before={name:(root/name).read_bytes() for name in [*patch.EDITS,patch.HEADER]}
            with self.assertRaisesRegex(ValueError,'Unsupported/partial'):
                patch.apply(root)
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)


if __name__=='__main__':unittest.main()
