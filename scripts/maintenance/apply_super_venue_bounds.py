#!/usr/bin/env python3
"""Add optional venue halfspaces to SUPER search and all generated corridors.

Recorded SUPER 2ad3419 layout; source-only, backed up and reversible. The motion
acceptance patch is independent. Profiles without venue_halfspaces are unchanged.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

VERSION='VENUE_BOUNDS_V1'
SEARCH_VERSION='VENUE_SEARCH_COMPLETE_V1'
REST_VERSION='VENUE_REST_COMMIT_V1'
HEADER='super_planner/include/super_core/demo_venue.hpp'
ASTAR_H='super_planner/include/path_search/astar.h'
ASTAR='super_planner/src/super_core/astar.cpp'
CORRIDOR_H='super_planner/include/super_core/corridor_generator.h'
CORRIDOR='super_planner/src/super_core/corridor_generator.cpp'
PLANNER='super_planner/src/super_core/super_planner.cpp'

HEADER_TEXT='''// VENUE_BOUNDS_V1: optional FC-centre bounds, independent of obstacle inflation.
#pragma once
#include <Eigen/Dense>
#include <utils/header/yaml_loader.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace super_utils {
class DemoVenue {
    Eigen::Matrix<double, Eigen::Dynamic, 4> planes_{0, 4};
public:
    DemoVenue() = default;
    explicit DemoVenue(const std::string &path) {
        yaml_loader::YamlLoader loader(path);
        std::vector<double> flat;
        loader.LoadParam("super_planner/venue_halfspaces", flat, std::vector<double>{});
        if (flat.empty()) return;
        if (flat.size() != 24) throw std::invalid_argument("Venue requires six world-frame halfspaces");
        planes_.resize(6, 4);
        for (int i=0; i<24; ++i) planes_(i/4, i%4)=flat[i];
        if (!planes_.allFinite()) throw std::invalid_argument("Nonfinite venue halfspace");
        Eigen::Matrix3d axes;
        for (int i=0; i<3; ++i) {
            axes.row(i)=planes_.row(2*i).head<3>();
            if ((planes_.row(2*i).head<3>() + planes_.row(2*i+1).head<3>()).norm() > 1e-8 ||
                planes_(2*i,3)+planes_(2*i+1,3) >= 0)
                throw std::invalid_argument("Venue plane pairs must enclose a nonempty box");
        }
        if (!(axes*axes.transpose()).isApprox(Eigen::Matrix3d::Identity(),1e-8))
            throw std::invalid_argument("Venue axes must be orthonormal");
        std::cout << "VENUE_BOUNDS_V1: search and corridors use six world-frame FC-centre halfspaces" << std::endl;
    }
    bool enabled() const { return planes_.rows() != 0; }
    bool contains(const Eigen::Vector3d &p) const {
        if (!enabled()) return true;
        return p.allFinite() && ((planes_.leftCols<3>()*p+planes_.col(3)).array() <= 1e-9).all();
    }
    template<class Matrix> void appendTo(Matrix &planes) const {
        if (!enabled()) return;
        const auto n=planes.rows();
        planes.conservativeResize(n+planes_.rows(),4);
        planes.bottomRows(planes_.rows())=planes_;
    }
};
}
'''

CLIP='''        // VENUE_BOUNDS_V1: intersect every corridor, including empty/fallback ones.
        super_utils::DemoVenue demo_venue_;
        bool applyDemoVenue(Polytope &polytope) const {
            if (!demo_venue_.enabled()) return true;
            auto planes=polytope.GetPlanes();
            demo_venue_.appendTo(planes);
            polytope.SetPlanes(planes);
            return true;
        }
'''

EDITS={
    ASTAR_H:[
        ('#include <ros_interface/ros_interface.hpp>','#include <ros_interface/ros_interface.hpp>\n#include <super_core/demo_venue.hpp> // VENUE_BOUNDS_V1',1),
        ('        PathSearchConfig cfg_;','        PathSearchConfig cfg_;\n        super_utils::DemoVenue demo_venue_;',1),
        ('        void setVisualProcessEn(const bool &en);',
         '        void setDemoVenue(const super_utils::DemoVenue &venue) { demo_venue_=venue; }\n\n        void setVisualProcessEn(const bool &en);',1),
    ],
    ASTAR:[
        ('    bool Astar::insideLocalMap(const rog_map::Vec3i &id_g) const {\n',
         '    bool Astar::insideLocalMap(const rog_map::Vec3i &id_g) const {\n        // VENUE_BOUNDS_V1: every normal/escape neighbor uses this predicate.\n        if (demo_venue_.enabled()) {\n            rog_map::Vec3f position;\n            globalIndexToPos(id_g,position);\n            if (!demo_venue_.contains(position)) return false;\n        }\n',1),
        ('                                           rog_map::vec_Vec3f &out_path, const double &time_out) {\n',
         '                                           rog_map::vec_Vec3f &out_path, const double &time_out) {\n        if (!demo_venue_.contains(start_pt) || !demo_venue_.contains(end_pt)) return INIT_ERROR;\n',1),
        ('        posToGlobalIndex(local_start_pt, start_idx);\n\n        GridNodePtr startPtr',
         '        posToGlobalIndex(local_start_pt, start_idx);\n        if (!insideLocalMap(start_idx)) return INIT_ERROR;\n\n        GridNodePtr startPtr',1),
    ],
    CORRIDOR_H:[
        ('#include <super_core/ciri.h>','#include <super_core/ciri.h>\n#include <super_core/demo_venue.hpp>',1),
        ('    class CorridorGenerator {\n    private:', '    class CorridorGenerator {\n    private:\n'+CLIP,1),
        ('        vec_Vec3f getLatestCloud() {',
         '        void setDemoVenue(const super_utils::DemoVenue &venue) { demo_venue_=venue; }\n\n        vec_Vec3f getLatestCloud() {',1),
    ],
    CORRIDOR:[
        ('            GenerateEmptyPolytope(path[0], dis, temp_poly);',
         '            if (!GenerateEmptyPolytope(path[0], dis, temp_poly)) return false;',1),
        ('    bool CorridorGenerator::GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope) {\n',
         '    bool CorridorGenerator::GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope) {\n        // VENUE_BOUNDS_V1: seeds and final corridor share the same box.\n        if (!demo_venue_.contains(pt)) return false;\n',1),
        ('                                                  Polytope & polytope){\n',
         '                                                  Polytope & polytope){\n        if (!demo_venue_.contains(pt)) return false;\n',1),
        ('    bool CorridorGenerator::GeneratePolytopeFromLine(Line &line, Polytope &polytope) {\n',
         '    bool CorridorGenerator::GeneratePolytopeFromLine(Line &line, Polytope &polytope) {\n        if (!demo_venue_.contains(line.first) || !demo_venue_.contains(line.second)) return false;\n',1),
        ('            polytope.SetSeedLine(Line{pt, pt});\n            return true;',
         '            polytope.SetSeedLine(Line{pt, pt});\n            return applyDemoVenue(polytope);',2),
        ('        polytope.SetSeedLine(Line{pt, pt});\n        return true;',
         '        polytope.SetSeedLine(Line{pt, pt});\n        return applyDemoVenue(polytope);',1),
        ('            polytope.SetSeedLine(line);\n            return true;',
         '            polytope.SetSeedLine(line);\n            return applyDemoVenue(polytope);',2),
    ],
    PLANNER:[
        ('        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);',
         '        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);\n        // VENUE_BOUNDS_V1: one immutable box for search and corridor constraints.\n        const super_utils::DemoVenue demo_venue(cfg_path);\n        astar_ptr_->setDemoVenue(demo_venue);\n        cg_ptr_->setDemoVenue(demo_venue);',1),
    ],
}

# Upgrade the deployed V1 venue patch without changing geometry or the motion
# patch. Only searches with an explicit finite venue lose upstream time cutoffs.
SEARCH_EDITS={
    ASTAR:[
        ('if (!cfg_.visual_process && (time_2 - time_1) > time_out)',
         'if (!demo_venue_.enabled() && !cfg_.visual_process && (time_2 - time_1) > time_out)',1),
        ('if ((time_2 - time_1) > time_out)',
         'if (!demo_venue_.enabled() && (time_2 - time_1) > time_out)',1),
        ('if (!cfg_.visual_process && (time_2 - time_1) > 0.2)',
         'if (!demo_venue_.enabled() && !cfg_.visual_process && (time_2 - time_1) > 0.2)',1),
        ('        ros_ptr_->error(" -- [A*] Point to point path cannot find path with iter num: {}, return.", num_iter);',
         '        if (demo_venue_.enabled()) {\n'
         '            ros_ptr_->warn(" -- [A*] VENUE_SEARCH_COMPLETE_V1 exhausted: map={} nodes={} elapsed_s={} start={} goal={} goal_occupied_raw={} goal_occupied_inflated={}",\n'
         '                md_.use_inf_map ? "inflated" : "probability", num_iter, time_2-time_1,\n'
         '                local_start_pt.transpose(), local_end_pt.transpose(),\n'
         '                map_ptr_->getGridType(local_end_pt)==OCCUPIED, map_ptr_->getInfGridType(local_end_pt)==OCCUPIED);\n'
         '        }\n'
         '        ros_ptr_->error(" -- [A*] Point to point path cannot find path with iter num: {}, return.", num_iter);',1),
    ],
    PLANNER:[
        ('        cg_ptr_->setDemoVenue(demo_venue);',
         '        cg_ptr_->setDemoVenue(demo_venue);\n'
         '        if (demo_venue.enabled()) ros_ptr_->info("VENUE_SEARCH_COMPLETE_V1: finite venue search; no elapsed-time cutoff");',1),
    ],
}


# From-rest planning has no moving splice to meet. Start its clock only when
# the full primary/backup pair is ready; moving replans keep their splice checks.
REST_COMMIT = """        // VENUE_REST_COMMIT_V1: a held vehicle has not executed optimization time.
        if (astar_ptr_->hasDemoVenue() &&
            (back_ret_code == SUCCESS || back_ret_code == FINISH || back_ret_code == NO_NEED)) {
            const double commit_WT = ros_ptr_->getSimTime();
            exp_traj_info.setTrajectory(commit_WT, exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            if (!back_traj_info.empty()) {
                back_traj_info.setTrajectory(commit_WT + back_traj_info.getStartTT(),
                    back_traj_info.getStartTT(), back_traj_info.posTraj(), back_traj_info.yawTraj());
                latest_replan.setBackupTraj(back_traj_info.posTraj());
                latest_replan.setBackupYawTraj(back_traj_info.yawTraj());
            }
            latest_replan.setExpTraj(exp_traj_info.posTraj());
            latest_replan.setExpYawTraj(exp_traj_info.yawTraj());
            ros_ptr_->info("VENUE_REST_COMMIT_V1: from-rest plan starts at commit; backup offset preserved");
        }
"""
REST_EDITS={
    ASTAR_H:[
        ('        void setDemoVenue(const super_utils::DemoVenue &venue) { demo_venue_=venue; }',
         '        bool hasDemoVenue() const { return demo_venue_.enabled(); } // VENUE_REST_COMMIT_V1\n'
         '        void setDemoVenue(const super_utils::DemoVenue &venue) { demo_venue_=venue; }',1),
    ],
    PLANNER:[
        ('        if (replan_total_t > cfg_.replan_forward_dt) {',
         '        if (!(astar_ptr_->hasDemoVenue() && last_exp_traj_info.empty()) &&\n'
         '            replan_total_t > cfg_.replan_forward_dt) {',1),
        ('        double new_traj_WT = replan_process_start_WT;',
         '        double new_traj_WT = (astar_ptr_->hasDemoVenue() && last_exp_traj_info.empty())\n'
         '            ? ros_ptr_->getSimTime() : replan_process_start_WT;',1),
        ('        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;\n',
         '        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;\n'+REST_COMMIT,1),
    ],
}


def patched(relative,text):
    if VERSION in text:
        if not all(text.count(new)==count for _,new,count in EDITS[relative]):
            raise ValueError('Incomplete or modified venue patch: '+relative)
    else:
        for old,new,count in EDITS[relative]:
            if text.count(old)!=count:raise ValueError('Unsupported source context; no files written: '+relative)
            text=text.replace(old,new)
    for marker,edits in ((SEARCH_VERSION,SEARCH_EDITS),(REST_VERSION,REST_EDITS)):
        upgrades=edits.get(relative,[])
        if not upgrades:continue
        if all(text.count(new)==count for _,new,count in upgrades):continue
        if marker in text:
            raise ValueError('Partial bounded-search upgrade; no files written: '+relative)
        if not all(text.count(old)==count for old,_,count in upgrades):
            raise ValueError('Unsupported/partial bounded-search upgrade; no files written: '+relative)
        for old,new,_ in upgrades:text=text.replace(old,new)
    return text


def apply(root,dry_run=False):
    root=Path(root).expanduser().resolve();changes={}
    for name in [*EDITS,HEADER]:
        path=root/name
        if path.is_symlink():raise ValueError('Refusing symlinked source: '+str(path))
        before=path.read_bytes() if path.exists() else None
        if name==HEADER:
            after=HEADER_TEXT.encode()
            if before is not None and before!=after:raise ValueError('Venue header differs; no files written')
        else:
            if before is None:raise ValueError('Missing source: '+name)
            after=patched(name,before.decode()).encode()
        if before!=after:changes[name]=(before,after)
    if not changes:
        print('Venue source patch already applied; rebuild if not yet done.');return None
    if dry_run:
        for name,(before,after) in changes.items():
            print(''.join(difflib.unified_diff((before or b'').decode().splitlines(True),after.decode().splitlines(True),name,name)),end='')
        return None
    backup=Path(tempfile.mkdtemp(prefix='venue_bounds_backup_',dir=root))
    record=dict(version=VERSION,root=str(root),files={})
    for name,(before,after) in changes.items():
        if before is not None:
            saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/name,saved)
        record['files'][name]=dict(before=hashlib.sha256(before).hexdigest() if before is not None else None,
                                  after=hashlib.sha256(after).hexdigest())
    (backup/'record.json').write_text(json.dumps(record,indent=2)+'\n')
    for name,(_,after) in changes.items():(root/name).write_bytes(after)
    print('Venue source patch applied. Backup:',backup)
    print('Rebuild super_planner before running a venue-bounded rehearsal.')
    return backup


def restore(backup):
    backup=Path(backup).expanduser().resolve();record=json.loads((backup/'record.json').read_text())
    if record.get('version')!=VERSION:raise ValueError('Unsupported backup')
    root=Path(record['root'])
    for name,hashes in record['files'].items():
        if name not in [*EDITS,HEADER]:raise ValueError('Unexpected backup source')
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=hashes['after']:
            raise ValueError('Source changed after patch; refusing restore: '+name)
        if hashes['before'] is not None and hashlib.sha256((backup/name).read_bytes()).hexdigest()!=hashes['before']:
            raise ValueError('Backup changed: '+name)
    for name,hashes in record['files'].items():
        if hashes['before'] is None:(root/name).unlink()
        else:shutil.copy2(backup/name,root/name)
    print('Original source restored; rebuild super_planner before restarting.')


def require_support(root):
    root=Path(root).expanduser().resolve()
    for name in EDITS:
        text=(root/name).read_text()
        if VERSION not in text or patched(name,text)!=text:
            raise ValueError('Apply apply_super_venue_bounds.py and rebuild super_planner first')
    if (root/HEADER).read_text()!=HEADER_TEXT:raise ValueError('Venue support header differs; rebuild after inspection')
    workspace=root.parent.parent
    for name in ('install/super_planner/lib/super_planner/fsm_node','install/lib/super_planner/fsm_node'):
        path=workspace/name
        if path.is_file():
            with path.open('rb') as handle:
                if handle.read(4)!=b'\x7fELF':continue
                handle.seek(0);previous=b'';found=set()
                while chunk:=handle.read(1024*1024):
                    for marker in (VERSION,SEARCH_VERSION,REST_VERSION):
                        if marker.encode() in previous+chunk:found.add(marker)
                    if len(found)==3:return path
                    previous=chunk[-max(len(VERSION),len(SEARCH_VERSION),len(REST_VERSION)):]
    raise ValueError('Installed fsm_node needs complete bounded-search support; rebuild super_planner before the rehearsal')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',type=Path,default=Path.home()/'super_ws/src/SUPER')
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--restore',type=Path)
    args=p.parse_args()
    restore(args.restore) if args.restore else apply(args.source_root,args.dry_run)


if __name__=='__main__':main()
