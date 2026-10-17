"""Pinned SUPER source patch, compiled acceptance bounds and unchanged targets."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/planning'),str(ROOT/'scripts/flight'),str(ROOT/'tests')]
import apply_super_motion_acceptance as patcher
import create_super_motion_acceptance_profile as profiles
from bounded_reference_preview import Plan,Reference,TakeoffFrame
from test_bounded_reference import trajectory

FIXTURES=ROOT/'tests/fixtures/super_motion_acceptance'


def source_tree(root):
    for target,fixture in ((patcher.CONFIG,'config.hpp'),(patcher.BACKUP,'backup_checks.cpp'),
                           (patcher.EXPLORATION,'exploration_checks.cpp')):
        path=Path(root)/target;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes((FIXTURES/fixture).read_bytes())


class Tests(unittest.TestCase):
    def test_source_patch_is_atomic_idempotent_and_restores_original_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root)
            original={name:(root/name).read_bytes() for name in patcher.EDITS}
            backup=patcher.apply(root)
            for relative in patcher.EDITS:self.assertIn(patcher.VERSION,(root/relative).read_text())
            self.assertIsNone(patcher.apply(root))
            patcher.restore(backup)
            for name,data in original.items():self.assertEqual((root/name).read_bytes(),data)

    def test_unsupported_file_does_not_partially_patch_other_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root)
            (root/patcher.EXPLORATION).write_text('different source')
            before={name:(root/name).read_bytes() for name in patcher.EDITS}
            with self.assertRaisesRegex(ValueError,'Unsupported source context'):patcher.apply(root)
            for name,data in before.items():self.assertEqual((root/name).read_bytes(),data)

    def test_restore_refuses_to_overwrite_later_source_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root);backup=patcher.apply(root)
            path=root/patcher.BACKUP;path.write_text(path.read_text()+'\n// later edit\n')
            with self.assertRaisesRegex(ValueError,'Source changed'):patcher.restore(backup)

    def test_profile_only_adds_acceptance_fields_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'source.yaml'
            data=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
            data['traj_opt']['boundary'].update(max_vel=2.,max_acc=1.)
            source.write_text(yaml.safe_dump(data));before=source.read_bytes()
            expected=deepcopy(data)
            expected['traj_opt']['boundary'].update(acceptance_max_vel=4.,acceptance_max_acc=2.)
            target=profiles.create(source)
            self.assertEqual(yaml.safe_load(target.read_text()),expected)
            self.assertEqual(source.read_bytes(),before)
            self.assertEqual(profiles.create(source),target)
            target.write_text('changed')
            with self.assertRaisesRegex(ValueError,'Existing acceptance profile differs'):profiles.create(source)

    def test_profile_support_requires_patched_source_and_installed_marker(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'src/SUPER';source_tree(root)
            with self.assertRaisesRegex(ValueError,'Apply apply_super'):profiles.require_support(root)
            patcher.apply(root)
            with self.assertRaisesRegex(ValueError,'rebuild'):profiles.require_support(root)
            binary=Path(folder)/'install/super_planner/lib/super_planner/fsm_node'
            binary.parent.mkdir(parents=True);binary.write_bytes(b'\x7fELF'+patcher.VERSION.encode())
            self.assertEqual(profiles.require_support(root),binary)

    def test_session_demo_profile_wires_acceptance_without_changing_base_yaml(self):
        import super_session as manager
        with tempfile.TemporaryDirectory() as folder,patch('pathlib.Path.home',return_value=Path(folder)):
            config_dir=Path(folder)/'super_ws/src/SUPER/super_planner/config'
            config_dir.mkdir(parents=True)
            data=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
            data['fsm']['click_height']=-6.
            data['traj_opt']['boundary'].update(max_vel=2.,max_acc=1.)
            text=yaml.safe_dump(data)
            (config_dir/'mid360_ground_diagnostic_ros2.yaml').write_text(text)
            source=config_dir/'mid360_demo_mission_ros2.yaml';source.write_text(text)
            session=manager.Session(manager.arguments(['planner','--planner-profile','demo']))
            with patch.object(profiles,'require_support') as support:
                target=session.planner_profile()
            support.assert_called_once_with(config_dir.parent.parent)
            self.assertEqual(source.read_text(),text)
            boundary=yaml.safe_load(target.read_text())['traj_opt']['boundary']
            self.assertEqual((boundary['max_vel'],boundary['max_acc']),(2.,1.))
            self.assertEqual((boundary['acceptance_max_vel'],boundary['acceptance_max_acc']),(4.,2.))

    def test_compiled_original_config_and_real_backup_guard_use_separate_bounds(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source_tree(root);patcher.apply(root)
            include=root/'include';(include/'utils/geometry').mkdir(parents=True)
            (include/'utils/header').mkdir(parents=True)
            (include/'utils/geometry/quadrotor_flatness.hpp').write_text(
                '#pragma once\nnamespace flatness { struct FlatnessMap { template<class... T> void reset(T...) {} }; }\n')
            (include/'utils/header/yaml_loader.hpp').write_text('''#pragma once
#include <map>
#include <string>
#include <vector>
namespace yaml_loader {
extern std::map<std::string,double> values;
struct YamlLoader {
 explicit YamlLoader(const std::string&) {}
 template<class T> void LoadParam(const std::string& key,T& value,const T& fallback) {
   value=values.count(key) ? static_cast<T>(values.at(key)) : fallback;
 }
}; }
''')
            header=root/patcher.CONFIG
            backup=(root/patcher.BACKUP).read_text()
            body=backup[backup.index('bool BackupTrajOpt::checkTrajMagnitudeBound'):]
            code='''#include <cassert>
#include <limits>
#include "'''+str(header)+'''"
std::map<std::string,double> yaml_loader::values;
#define YELLOW ""
#define RESET ""
struct Trajectory {
 double velocity,acceleration;bool cleared=false;
 double getMaxVelRate() const {return velocity;}
 double getMaxAccRate() const {return acceleration;}
 void clear() {cleared=true;}
};
struct Log {void warn(const char*) {}};
struct BackupTrajOpt {
 traj_opt::Config cfg_;
 bool checkTrajMagnitudeBound(Trajectory&);
};
'''+body+'''
bool explorationAccept(traj_opt::Config cfg_,double velocity,double acceleration) {
 Log log;Log* ros_ptr_=&log;Trajectory traj{velocity,acceleration};int ret=1;double minCostFunctional=0;
'''+patcher.EXP_FINAL+'''
 return ret>=0 && !traj.cleared && std::isfinite(minCostFunctional);
}
int main() {
 auto& values=yaml_loader::values;
 values["traj_opt/boundary/max_vel"]=2.;values["traj_opt/boundary/max_acc"]=1.;
 traj_opt::Config legacy("unused","backup_traj");
 assert(std::abs(legacy.velocityViolationLimit()-.1)<1e-12);
 assert(std::abs(legacy.accelerationViolationLimit()-.05)<1e-12);
 assert(legacy.accelerationAcceptanceBound()==1.2);
 values["traj_opt/boundary/acceptance_max_vel"]=4.;
 values["traj_opt/boundary/acceptance_max_acc"]=2.;
 traj_opt::Config cfg("unused","backup_traj");
 assert(cfg.max_vel==2. && cfg.max_acc==1.);
 assert(cfg.velocityViolationLimit()==12. && cfg.accelerationViolationLimit()==3.);
 assert(.149380025752899 < cfg.accelerationViolationLimit());
 assert(4.*4.-cfg.max_vel*cfg.max_vel <= cfg.velocityViolationLimit());
 assert(4.001*4.001-cfg.max_vel*cfg.max_vel > cfg.velocityViolationLimit());
 assert(2.*2.-cfg.max_acc*cfg.max_acc <= cfg.accelerationViolationLimit());
 assert(2.001*2.001-cfg.max_acc*cfg.max_acc > cfg.accelerationViolationLimit());
 cfg.penna_vel=cfg.penna_acc=1.;BackupTrajOpt optimizer{cfg};
 Trajectory good{3.5,1.75},fast{4.01,1.},accel{2.,2.01};
 assert(optimizer.checkTrajMagnitudeBound(good));
 assert(!optimizer.checkTrajMagnitudeBound(fast));
 assert(!optimizer.checkTrajMagnitudeBound(accel));
 assert(explorationAccept(cfg,4.,2.));
 assert(!explorationAccept(cfg,4.01,1.));
 assert(!explorationAccept(cfg,2.,2.01));
 assert(!explorationAccept(cfg,std::numeric_limits<double>::infinity(),1.));
 values["traj_opt/boundary/acceptance_max_vel"]=1.;
 bool rejected=false;try {traj_opt::Config bad("unused","backup_traj");} catch(const std::invalid_argument&) {rejected=true;}
 assert(rejected);
}
'''
            cpp=root/'check.cpp';cpp.write_text(code)
            result=subprocess.run(['g++','-std=c++14','-I'+str(include),str(cpp),'-o',str(root/'check')],
                                  text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(root/'check')],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('target=2/1 accept=4/2',result.stdout)

    def follower(self,speed,acceleration,explicit=True,observe=False):
        frame=TakeoffFrame([0,0,0],[0,0,0,1],max_forward=5.)
        kwargs=dict(acceptance_speed=4.,acceptance_acceleration=2.) if explicit else {}
        reference=Reference(frame,2.,1.,'observe' if observe else 'strict',**kwargs)
        msg=trajectory(distance=0.)
        msg.coef_pos_x=[0.,0.,0.,0.,0.,acceleration/2,speed,0.]
        reference.add_plan(msg)
        return reference,msg

    def test_reference_accepts_target_excess_without_changing_targets(self):
        ref,msg=self.follower(3.5,1.5)
        p,v,a=Plan(msg).evaluate(0.)
        self.assertTrue(ref.command(100_000_000_000,p,v,a,p,[0,0,0,1]))
        self.assertEqual((ref.max_speed,ref.max_acceleration),(2.,1.))
        self.assertEqual(ref.motion_summary['speed_exceedances'],1)
        self.assertEqual(ref.motion_summary['acceptance_speed_exceedances'],0)

    def test_reference_rejects_only_above_new_bounds_in_strict_mode(self):
        for speed,acceleration in ((4.,2.),(4.001,1.),(2.,2.001)):
            ref,msg=self.follower(speed,acceleration);p,v,a=Plan(msg).evaluate(0.)
            if speed<=4. and acceleration<=2.:
                self.assertTrue(ref.command(100_000_000_000,p,v,a,p,[0,0,0,1]))
            else:
                with self.assertRaisesRegex(ValueError,'derivative exceeds 4/2'):
                    ref.command(100_000_000_000,p,v,a,p,[0,0,0,1])

    def test_default_reference_and_observe_semantics_are_preserved(self):
        ref,msg=self.follower(2.1,1.,explicit=False);p,v,a=Plan(msg).evaluate(0.)
        with self.assertRaises(ValueError):ref.command(100_000_000_000,p,v,a,p,[0,0,0,1])
        ref,msg=self.follower(4.1,2.1,observe=True);p,v,a=Plan(msg).evaluate(0.)
        self.assertTrue(ref.command(100_000_000_000,p,v,a,p,[0,0,0,1]))
        self.assertEqual(ref.motion_summary['acceptance_speed_exceedances'],1)


if __name__=='__main__':unittest.main()
