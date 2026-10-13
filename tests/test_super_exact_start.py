"""Guarded patch/rollback and compiled exact C++ start-block behavior."""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('exact',ROOT/'scripts/maintenance/apply_super_exact_start.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def source(self,root,content):
        path=root/m.RELATIVE;path.parent.mkdir(parents=True);path.write_bytes(content)
        return path

    def test_backup_patch_idempotence_and_guarded_rollback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'repo';source=self.source(root,b'// retain unrelated edits\n'+m.OLD+b'// trailing edits\n')
            before=source.read_bytes()
            with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,stdout=m.REVISION+'\n')):
                backup=m.patch(root,Path(tmp)/'backups')
                self.assertEqual(source.read_bytes(),before.replace(m.OLD,m.NEW))
                self.assertEqual((backup/source.name).read_bytes(),before)
                self.assertIsNone(m.patch(root,Path(tmp)/'backups'))
            expected=source.read_bytes()
            source.write_bytes(expected+b'// subsequent change\n')
            with self.assertRaisesRegex(ValueError,'later edits'):m.restore(backup)
            source.write_bytes(expected);m.restore(backup);self.assertEqual(source.read_bytes(),before)
            m.restore(backup)

    def test_revision_and_source_mismatches_do_not_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'repo';source=self.source(root,m.OLD)
            for revision,content in (('different',m.OLD),(m.REVISION,m.OLD+m.OLD),(m.REVISION,b'local altered block')):
                source.write_bytes(content)
                with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,stdout=revision)):
                    with self.assertRaises(ValueError):m.patch(root,Path(tmp)/'backups')
                self.assertEqual(source.read_bytes(),content)
                self.assertFalse((Path(tmp)/'backups').exists())
            source.write_bytes(m.OLD)
            copy=m.shutil.copy2
            changed=m.OLD+b'// concurrent edit\n'
            def edit_during_backup(src,dst):
                copy(src,dst);source.write_bytes(changed)
            with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,stdout=m.REVISION)), \
                 patch.object(m.shutil,'copy2',side_effect=edit_during_backup):
                with self.assertRaisesRegex(ValueError,'changed during backup'):
                    m.patch(root,Path(tmp)/'race_backups')
            self.assertEqual(source.read_bytes(),changed)

    def test_restore_applied_requires_matching_backup_and_leaves_original_alone(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'repo';backups=Path(tmp)/'backups'
            source=self.source(root,b'// local edits\n'+m.OLD)
            original=source.read_bytes()
            self.assertFalse(m.restore_applied(root,backups))
            with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,stdout=m.REVISION)):
                m.patch(root,backups)
            patched=source.read_bytes()
            source.write_bytes(patched+b'// later edits\n')
            with self.assertRaisesRegex(ValueError,'No matching backup'):
                m.restore_applied(root,backups)
            self.assertEqual(source.read_bytes(),patched+b'// later edits\n')
            source.write_bytes(patched)
            self.assertTrue(m.restore_applied(root,backups))
            self.assertEqual(source.read_bytes(),original)
            self.assertFalse(m.restore_applied(root,backups))

    def test_compiled_block_preserves_pose_and_rejects_invalid_map_starts(self):
        # Execute the exact patch statements, not a reimplementation. Mock map
        # classifications isolate the boundary logic; this is not a full SUPER build.
        prefix=r'''
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#ifdef EXACT_START_WITH_EIGEN
#include <Eigen/Core>
using Vec3f=Eigen::Vector3d;
Vec3f point(double x,double y,double z) {return Vec3f(x,y,z);}
bool same(const Vec3f& a,const Vec3f& b) {return (a-b).norm()==0.;}
#else
struct Vec3f { std::array<double,3> p;
  bool allFinite() const { for(double v:p) if(!std::isfinite(v)) return false; return true; }
};
Vec3f point(double x,double y,double z) {return {{x,y,z}};}
bool same(const Vec3f& a,const Vec3f& b) {return a.p==b.p;}
#endif
enum GridType { UNDEFINED, UNKNOWN, OUT_OF_MAP, OCCUPIED, KNOWN_FREE, FRONTIER };
enum SUPER_RET_CODE { SUPER_NO_START_POINT=1 };
constexpr int FAILED=-1;
struct Map { GridType type=KNOWN_FREE; int calls=0;
  GridType getGridType(const Vec3f&) { ++calls; return type; }
};
struct Ros { template<typename... Args> void error(Args...) {} template<typename... Args> void info(Args...) {} };
struct Replan { int code=0; Vec3f selected{};
  void setRetCode(int c) {code=c;} void setLocalStartP(Vec3f p) {selected=p;}
};
struct Planner { Map map; Ros ros; Map* map_ptr_=&map; Ros* ros_ptr_=&ros;
  struct {Vec3f p;} robot_state_; Replan latest_replan;
  int run() {
'''
        suffix=r'''
    return 0;
  }
};
int main() {
  for(auto type:{KNOWN_FREE,UNKNOWN}) {
    Planner p; p.map.type=type; p.robot_state_.p=point(.0147,-.0943,-.1261);
    assert(p.run()==0); assert(p.map.calls==1);
    assert(same(p.latest_replan.selected,p.robot_state_.p));
  }
  for(auto type:{OCCUPIED,OUT_OF_MAP,UNDEFINED,FRONTIER}) {
    Planner p; p.map.type=type; p.robot_state_.p=point(.0147,-.0943,-.1261);
    assert(p.run()==FAILED); assert(p.latest_replan.code==SUPER_NO_START_POINT);
  }
  Planner p; p.robot_state_.p=point(std::numeric_limits<double>::quiet_NaN(),0.,0.);
  assert(p.run()==FAILED); assert(p.map.calls==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'guard.cpp';binary=Path(tmp)/'guard'
            source.write_text(prefix+m.NEW.decode()+suffix)
            command=['g++','-std=c++14','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)]
            eigen=os.environ.get('EXACT_START_EIGEN_INCLUDE')
            if eigen:
                self.assertTrue((Path(eigen)/'Eigen/Core').is_file())
                command += ['-DEXACT_START_WITH_EIGEN','-I'+eigen]
            subprocess.run(command,check=True)
            subprocess.run([str(binary)],check=True)


if __name__=='__main__':unittest.main()
