#!/usr/bin/env python3
"""Add optional venue halfspaces to SUPER search and all generated corridors.

Recorded SUPER 2ad3419 layout; source-only, backed up and reversible. The motion
acceptance patch is independent. Venue search remains opt-in. The ROG-Map height
query correction uses the already-inflated virtual bounds for every profile.
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
MAP_VERSION='VIRTUAL_HEIGHT_QUERY_V1'
COMMIT_VERSION='VENUE_CANDIDATE_CHECK_V1'
HEIGHT_VERSION='VIRTUAL_HEIGHT_CORRIDOR_V1'
GRID_VERSION='VENUE_GRID_ANCHOR_V1'
START_VERSION='VENUE_REST_START_V1'
MAP_DOMAIN_VERSION='VENUE_MAP_HEIGHT_V1'
INF_MAP='rog_map/src/rog_map/inf_map.cpp'
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


# A corridor constraint is a soft optimizer penalty. Validate the resulting
# polynomial before publication; subdivision certifies the whole interval,
# including peaks between command samples. Uncertain candidates are rejected.
HEADER_V1_TEXT=HEADER_TEXT
TRAJECTORY_CHECK_TEXT='''    static bool nonpositiveBernstein(const std::vector<double> &b, int depth=0) {
        double upper=-INFINITY;
        for (double value:b) {
            if (!std::isfinite(value)) return false;
            upper=std::max(upper,value);
        }
        if (upper<=1e-9) return true;
        if (b.front()>1e-9 || b.back()>1e-9 || depth==24) return false;
        const int n=static_cast<int>(b.size())-1;
        auto work=b;std::vector<double> left(n+1),right(n+1);
        left[0]=work[0];right[n]=work[n];
        for (int level=1;level<=n;++level) {
            for (int i=0;i<=n-level;++i) work[i]=.5*(work[i]+work[i+1]);
            left[level]=work[0];right[n-level]=work[n-level];
        }
        return nonpositiveBernstein(left,depth+1) && nonpositiveBernstein(right,depth+1);
    }
    template<class Trajectory> bool containsTrajectory(const Trajectory &trajectory) const {
        if (!enabled()) return true;
        if (trajectory.getPieceNum()==0) return false;
        for (int piece=0;piece<trajectory.getPieceNum();++piece) {
            const auto &coeff=trajectory[piece].getCoeffMat();
            const double duration=trajectory[piece].getDuration();
            if (coeff.rows()!=3 || coeff.cols()==0 || !coeff.allFinite() ||
                !std::isfinite(duration) || duration<=0) return false;
            const int n=static_cast<int>(coeff.cols())-1;
            for (int face=0;face<planes_.rows();++face) {
                std::vector<double> power(n+1),bernstein(n+1);
                double scale=1;
                for (int i=0;i<=n;++i) {
                    power[i]=planes_.row(face).head<3>().dot(coeff.col(n-i))*scale;
                    scale*=duration;
                }
                power[0]+=planes_(face,3);
                for (int k=0;k<=n;++k) {
                    double ratio=1;
                    for (int i=0;i<=k;++i) {
                        if (i) ratio*=double(k-i+1)/double(n-i+1);
                        bernstein[k]+=power[i]*ratio;
                    }
                }
                if (!nonpositiveBernstein(bernstein)) return false;
            }
        }
        return true;
    }
'''
HEADER_TEXT=HEADER_TEXT.replace('    template<class Matrix>',TRAJECTORY_CHECK_TEXT+'    template<class Matrix>')

HEADER_PRE_MAP_TEXT=HEADER_TEXT
MAP_HEIGHT_TEXT='''    void intersectMapHeight(double floor, double ceiling) {
        if (!enabled()) return;
        if (!std::isfinite(floor) || !std::isfinite(ceiling) || ceiling-floor<=2e-6)
            throw std::invalid_argument("Invalid effective map height interval");
        // Map queries treat equality as occupied. A micrometre numerical guard
        // keeps the closed polynomial certificate inside that open interval.
        const auto n=planes_.rows();
        planes_.conservativeResize(n+2,4);
        planes_.row(n)<<0,0,1,-(ceiling-1e-6);
        planes_.row(n+1)<<0,0,-1,floor+1e-6;
        std::cout << "VENUE_MAP_HEIGHT_V1: search, all corridors and complete candidate polynomials intersect effective map floor/ceiling="
                  << floor << "/" << ceiling << std::endl;
    }
'''
HEADER_TEXT=HEADER_TEXT.replace('    bool enabled() const',MAP_HEIGHT_TEXT+'    bool enabled() const')


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
    INF_MAP:[
        ('        if (pos.z() >= cfg_.virtual_ceil_height - cfg_.inflation_resolution * (1 + cfg_.inflation_step) ||\n'
         '            pos.z() <= cfg_.virtual_ground_height + cfg_.inflation_resolution * (1 + cfg_.inflation_step)) {',
         '        // VENUE_BOUNDS_V1 / VIRTUAL_HEIGHT_QUERY_V1: Config already inflates these bounds.\n'
         '        // Keep obstacle counters and the closed virtual boundary unchanged.\n'
         '        if (pos.z() >= cfg_.virtual_ceil_height ||\n'
         '            pos.z() <= cfg_.virtual_ground_height) {',1),
        ('        std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;',
         '        std::cout << "VIRTUAL_HEIGHT_QUERY_V1: use configured inflated floor/ceiling once" << std::endl;\n'
         '        std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;',1),
    ],
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



CANDIDATE_CHECK='''        // VENUE_CANDIDATE_CHECK_V1: never publish a soft-constraint violation.
        if ((back_ret_code == SUCCESS || back_ret_code == FINISH || back_ret_code == NO_NEED) &&
            (!astar_ptr_->trajectoryInsideDemoVenue(exp_traj_info.posTraj()) ||
             (!back_traj_info.empty() && !astar_ptr_->trajectoryInsideDemoVenue(back_traj_info.posTraj())))) {
            ros_ptr_->warn("VENUE_CANDIDATE_CHECK_V1: candidate outside venue; retain previous command and replan");
            return FAILED;
        }

'''
COMMIT_EDITS={
    ASTAR_H:[
        ('        void setVisualProcessEn(const bool &en);',
         '        void setVisualProcessEn(const bool &en);\n'
         '        // VENUE_CANDIDATE_CHECK_V1\n'
         '        template<class Trajectory> bool trajectoryInsideDemoVenue(const Trajectory &trajectory) const {\n'
         '            return demo_venue_.containsTrajectory(trajectory);\n'
         '        }',1),
    ],
    PLANNER:[
        ('        if (back_ret_code == SUCCESS) {',
         CANDIDATE_CHECK+'        if (back_ret_code == SUCCESS) {',2),
    ],
}


# The same effective virtual bounds feed corridor construction. Keep the radius
# inset on ordinary crop boxes, but do not inset an already-inflated map face.
HEIGHT_EDITS={
    CORRIDOR:[
        ('        virtual_ceil_height_ = virtual_ceil_height - robot_r;',
         '        std::cout << "VIRTUAL_HEIGHT_CORRIDOR_V1: configured virtual bounds inset once" << std::endl;\n'
         '        virtual_ceil_height_ = virtual_ceil_height;',1),
        ('        virtual_groud_height_ = virtual_groud_height + robot_r;',
         '        virtual_groud_height_ = virtual_groud_height;',1),
        ('        box_min.z() += robot_r_;\n        box_max.z() -= robot_r_;',
         '        if (box_min.z() > virtual_groud_height_) box_min.z() += robot_r_;\n'
         '        if (box_max.z() < virtual_ceil_height_) box_max.z() -= robot_r_;',2),
    ],
}

# A valid continuous point can quantize to a centre outside a rotated venue.
# Join it to an admissible neighbouring centre instead of changing the venue.
GRID_ANCHOR_TEXT='''    bool Astar::anchorVenueEndpoint(const rog_map::Vec3f &point,
                                      rog_map::Vec3f &anchor) {
        anchor=point;
        if (!demo_venue_.enabled()) return true;
        if (!demo_venue_.contains(point)) return false;
        rog_map::Vec3i id;posToGlobalIndex(point,id);
        rog_map::Vec3f snapped;globalIndexToPos(id,snapped);
        if (demo_venue_.contains(snapped)) return true;
        if (!map_ptr_->insideLocalMap(point)) return false;
        // Without unknown inflation, InfMap classifies every nonoccupied cell
        // as known free; its separate isUnknownInflate API would throw.
        const bool check_unknown=md_.unknown_as_occ &&
            (!md_.use_inf_map || map_ptr_->getMapConfig().unk_inflation_en);
        double nearest=INFINITY;
        for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
            const rog_map::Vec3i candidate_id=id+rog_map::Vec3i(x,y,z);
            if (!insideLocalMap(candidate_id)) continue;
            rog_map::Vec3f candidate;globalIndexToPos(candidate_id,candidate);
            if (!map_ptr_->insideLocalMap(candidate)) continue;
            auto type=md_.use_inf_map ? map_ptr_->getInfGridType(candidate) : map_ptr_->getGridType(candidate);
            if (type==OCCUPIED || type==OUT_OF_MAP || (md_.unknown_as_occ && type==UNKNOWN)) continue;
            if (md_.use_prob_map && md_.use_inf_neighbor && neighborHaveOne(OCCUPIED,candidate_id)) continue;
            const double distance=(candidate-point).squaredNorm();
            if (distance>=nearest || !map_ptr_->isLineFree(point,candidate,md_.use_inf_map,check_unknown)) continue;
            nearest=distance;anchor=candidate;
        }
        if (!std::isfinite(nearest)) return false;
        ros_ptr_->info("VENUE_GRID_ANCHOR_V1: continuous endpoint={} outside grid centre={} connected inside centre={}",
            point.transpose(),snapped.transpose(),anchor.transpose());
        return true;
    }

'''
GRID_EDITS={
    ASTAR_H:[
        ('        bool neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id);',
         '        bool anchorVenueEndpoint(const rog_map::Vec3f &point, rog_map::Vec3f &anchor); // VENUE_GRID_ANCHOR_V1\n\n'
         '        bool neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id);',1),
        ('        void setVisualProcessEn(const bool &en);',
         '        bool pointInsideDemoVenue(const rog_map::Vec3f &point) const { return demo_venue_.contains(point); }\n\n'
         '        void setVisualProcessEn(const bool &en);',1),
    ],
    ASTAR:[
        ('    RET_CODE Astar::pointToPointPathSearch(',GRID_ANCHOR_TEXT+'    RET_CODE Astar::pointToPointPathSearch(',1),
        ('        local_end_pt = end_pt;\n',
         '        local_end_pt = end_pt;\n'
         '        if (!anchorVenueEndpoint(start_pt,local_start_pt) ||\n'
         '            !anchorVenueEndpoint(end_pt,local_end_pt)) return NO_PATH;\n',1),
        ('        if (!insideLocalMap(start_pt)) {',
         '        if (!insideLocalMap(local_start_pt)) {',1),
        ('        if (!insideLocalMap(end_pt)) {',
         '        if (!insideLocalMap(local_end_pt)) {',1),
        ('        if (!insideLocalMap(start_pt) ||\n            !map_ptr_->insideLocalMap(start_pt)) {',
         '        rog_map::Vec3f venue_start_pt;\n'
         '        if (!anchorVenueEndpoint(start_pt,venue_start_pt)) return NO_PATH;\n'
         '        if (!insideLocalMap(venue_start_pt) ||\n            !map_ptr_->insideLocalMap(venue_start_pt)) {',1),
        ('        rog_map::Vec3f local_start_pt = start_pt;',
         '        rog_map::Vec3f local_start_pt = venue_start_pt;',1),
        ('if (!map_ptr_->getNearestCellNot(OCCUPIED, start_pt, local_start_pt,3.0))',
         'if (!map_ptr_->getNearestCellNot(OCCUPIED, venue_start_pt, local_start_pt,3.0))',1),
        ('        rog_map::Vec3i start_idx;\n        posToGlobalIndex(local_start_pt, start_idx);',
         '        rog_map::Vec3f admissible_start;\n'
         '        if (!anchorVenueEndpoint(local_start_pt,admissible_start)) return NO_PATH;\n'
         '        local_start_pt=admissible_start;\n'
         '        rog_map::Vec3i start_idx;\n        posToGlobalIndex(local_start_pt, start_idx);',1),
    ],
    PLANNER:[
        ('        if(ret_code == INIT_ERROR){\n            gi_.goal_valid = false;\n            return false;\n        }',
         '        if(ret_code == INIT_ERROR){\n'
         '            // VENUE_GRID_ANCHOR_V1: a start/search failure does not invalidate an in-venue goal.\n'
         '            if (!astar_ptr_->hasDemoVenue() || !astar_ptr_->pointInsideDemoVenue(goal))\n'
         '                gi_.goal_valid = false;\n'
         '            else ros_ptr_->warn("VENUE_GRID_ANCHOR_V1: search initialization failed; retaining unfinished goal");\n'
         '            return false;\n        }',1),
    ],
}

# Grid centres guide search; they must not replace the physical initial state.
# This applies only to the explicit demonstration venue. Obstacle/corridor and
# primary/backup trajectory checks still run from the retained measured start.
REST_START_OLD = """        /// 1) First, shift the start_point to free space.
        Vec3f local_star_pt;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
"""
REST_START_NEW = """        // VENUE_REST_START_V1: preserve the measured continuous rest start.
        Vec3f local_star_pt;
        bool start_valid = false;
        if (astar_ptr_->hasDemoVenue()) {
            local_star_pt = robot_state_.p;
            if (local_star_pt.allFinite() && map_ptr_->insideLocalMap(local_star_pt) &&
                astar_ptr_->pointInsideDemoVenue(local_star_pt)) {
                const auto type = map_ptr_->getGridType(local_star_pt);
                start_valid = type != GridType::OCCUPIED && type != GridType::OUT_OF_MAP;
            }
            if (start_valid)
                ros_ptr_->info("VENUE_REST_START_V1: measured start retained={} (no grid-centre relocation)",
                               local_star_pt.transpose());
            else
                ros_ptr_->warn("VENUE_REST_START_V1: measured start occupied, invalid, or outside map/venue; no relocation");
        } else {
            start_valid = map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0);
        }
        if (!start_valid) {
"""
START_EDITS={PLANNER:[(REST_START_OLD,REST_START_NEW,1)]}


# Use the map's initialized, quantized interval, not the source YAML heights.
# The same extra planes clip empty fallback corridors and certify both polynomials.
MAP_DOMAIN_EDITS={PLANNER:[(
    '        const super_utils::DemoVenue demo_venue(cfg_path);',
    '        super_utils::DemoVenue demo_venue(cfg_path);\n'
    '        // VENUE_MAP_HEIGHT_V1: effective map height is part of the admissible domain.\n'
    '        demo_venue.intersectMapHeight(rog_map_cfg.virtual_ground_height, rog_map_cfg.virtual_ceil_height);',1)]}


def patched(relative,text):
    # Unwrap the latest layer before validating earlier exact source edits.
    domain_edits=MAP_DOMAIN_EDITS.get(relative,[])
    if domain_edits and all(text.count(new)==count for _,new,count in domain_edits):
        for old,new,_ in reversed(domain_edits):text=text.replace(new,old)
    elif MAP_DOMAIN_VERSION in text:
        raise ValueError('Partial map-height domain upgrade; no files written: '+relative)
    # Validate an upgraded file against the original guarded contexts too.
    grid_upgrades=GRID_EDITS.get(relative,[])
    grid_applied=bool(grid_upgrades) and all(text.count(new)==count for _,new,count in grid_upgrades)
    if grid_applied:
        for old,new,_ in reversed(grid_upgrades):text=text.replace(new,old)
    elif GRID_VERSION in text:
        raise ValueError('Partial venue grid-anchor upgrade; no files written: '+relative)
    if VERSION in text:
        if not all(text.count(new)==count for _,new,count in EDITS[relative]):
            raise ValueError('Incomplete or modified venue patch: '+relative)
    else:
        for old,new,count in EDITS[relative]:
            if text.count(old)!=count:raise ValueError('Unsupported source context; no files written: '+relative)
            text=text.replace(old,new)
    for marker,edits in ((SEARCH_VERSION,SEARCH_EDITS),(REST_VERSION,REST_EDITS),(COMMIT_VERSION,COMMIT_EDITS),(HEIGHT_VERSION,HEIGHT_EDITS),(START_VERSION,START_EDITS)):
        upgrades=edits.get(relative,[])
        if not upgrades:continue
        if all(text.count(new)==count for _,new,count in upgrades):continue
        if marker in text:
            raise ValueError('Partial bounded-search upgrade; no files written: '+relative)
        if not all(text.count(old)==count for old,_,count in upgrades):
            raise ValueError('Unsupported/partial bounded-search upgrade; no files written: '+relative)
        for old,new,_ in upgrades:text=text.replace(old,new)
    for old,new,count in grid_upgrades:
        if text.count(old)!=count:raise ValueError('Unsupported grid-anchor source context; no files written: '+relative)
        text=text.replace(old,new)
    for old,new,count in domain_edits:
        if text.count(old)!=count:raise ValueError('Unsupported map-height domain context: '+relative)
        text=text.replace(old,new)
    return text


def apply(root,dry_run=False):
    root=Path(root).expanduser().resolve();changes={}
    for name in [*EDITS,HEADER]:
        path=root/name
        if path.is_symlink():raise ValueError('Refusing symlinked source: '+str(path))
        before=path.read_bytes() if path.exists() else None
        if name==HEADER:
            after=HEADER_TEXT.encode()
            if before is not None and before not in (after,HEADER_V1_TEXT.encode(),HEADER_PRE_MAP_TEXT.encode()):raise ValueError('Venue header differs; no files written')
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
    print('Rebuild rog_map and super_planner before running a venue-bounded rehearsal.')
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
    print('Original source restored; rebuild rog_map and super_planner before restarting.')


def require_support(root):
    root=Path(root).expanduser().resolve()
    for name in EDITS:
        text=(root/name).read_text()
        if VERSION not in text or patched(name,text)!=text:
            raise ValueError('Apply apply_super_venue_bounds.py and rebuild rog_map and super_planner first')
    if (root/HEADER).read_text()!=HEADER_TEXT:raise ValueError('Venue support header differs; rebuild after inspection')
    workspace=root.parent.parent
    for name in ('install/super_planner/lib/super_planner/fsm_node','install/lib/super_planner/fsm_node'):
        path=workspace/name
        if path.is_file():
            with path.open('rb') as handle:
                if handle.read(4)!=b'\x7fELF':continue
                handle.seek(0);previous=b'';found=set()
                while chunk:=handle.read(1024*1024):
                    for marker in (VERSION,SEARCH_VERSION,REST_VERSION,MAP_VERSION,COMMIT_VERSION,HEIGHT_VERSION,GRID_VERSION,START_VERSION,MAP_DOMAIN_VERSION):
                        if marker.encode() in previous+chunk:found.add(marker)
                    if len(found)==9:return path
                    previous=chunk[-max(len(VERSION),len(SEARCH_VERSION),len(REST_VERSION),len(MAP_VERSION),len(COMMIT_VERSION),len(HEIGHT_VERSION),len(GRID_VERSION),len(START_VERSION),len(MAP_DOMAIN_VERSION)):]
    raise ValueError('Installed fsm_node needs complete bounded-search, height-query, continuous rest-start and map-height domain support; rebuild rog_map and super_planner before the rehearsal')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',type=Path,default=Path.home()/'super_ws/src/SUPER')
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--restore',type=Path)
    args=p.parse_args()
    restore(args.restore) if args.restore else apply(args.source_root,args.dry_run)


if __name__=='__main__':main()
