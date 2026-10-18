    void CorridorGenerator::getSeedBBox(const Vec3f &p1, const Vec3f &p2, Vec3f &box_min, Vec3f &box_max) {
        box_min = p1.cwiseMin(p2);
        box_max = p1.cwiseMax(p2);
        box_min -= Vec3f(bound_dis_, bound_dis_, bound_dis_);
        box_max += Vec3f(bound_dis_, bound_dis_, bound_dis_);
//        box_min.z() = std::max(box_min.z(), virtual_groud_height_);
//        box_max.z() = std::min(box_max.z(), virtual_ceil_height_);
    }

    bool CorridorGenerator::GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope) {
        Eigen::Vector3d box_max, box_min;
        vec_E<Vec3f> pc;
        getSeedBBox(pt, pt, box_min, box_max);
        // TODO the box did not consider the robot_r
        map_ptr_->boundBoxByLocalMap(box_min, box_max);
        map_ptr_->boxSearch(box_min, box_max, OCCUPIED, pc);
        box_min.z() += robot_r_;
        box_max.z() -= robot_r_;
        MatD4f planes;
        Eigen::Vector3d a = pt, b = pt;
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;
        bd(0, 3) = -box_max.x();
        bd(1, 3) = box_min.x();
        bd(2, 3) = -box_max.y();
        bd(3, 3) = box_min.y();
        bd(4, 3) = -box_max.z();
        bd(5, 3) = box_min.z();
        // 将vector放到Eigen里，准备开始分解
        if (pc.empty()) {
            // 障碍物点云为空，直接返回一个方块
            // Ax + By + Cz + D = 0
            planes.resize(6, 4);
            planes.row(0) << 1, 0, 0, -box_max.x();
            planes.row(1) << 0, 1, 0, -box_max.y();
            planes.row(2) << 0, 0, 1, -box_max.z();
            planes.row(3) << -1, 0, 0, box_min.x();
            planes.row(4) << 0, -1, 0, box_min.y();
            planes.row(5) << 0, 0, -1, box_min.z();
            polytope.SetPlanes(planes);
            polytope.SetSeedLine(Line{pt, pt});
            return true;
        }
        latest_pc.insert(latest_pc.end(), pc.begin(), pc.end());
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc[0].data(), 3, pc.size());
        rog_map::TimeConsuming tc("emvp", false);
        RET_CODE success = ciri_->comvexDecomposition(bd, pp, a, b);
        double dt = tc.stop();
        if (success == SUCCESS) {
            ciri_cnt++;
            ciri_t += dt;
            ciri_->getPolytope(polytope);
            polytope.SetSeedLine(Line{pt, pt});
            return true;
        } else {
            cout << YELLOW << " -- [SUPER] CSpaceFiri failed." << RESET << endl;
            cout << YELLOW << "\t box_min =" << box_min.transpose() << endl;
            cout << YELLOW << "\t box_max = " << box_max.transpose() << endl;
            cout << YELLOW << "\t seed pt = " << pt.transpose() << endl;
            polytope.Reset();
            return false;
        }

    }

    bool CorridorGenerator::GenerateEmptyPolytope(const super_utils::Vec3f &pt,
                                                  const double & dis,
                                                  Polytope & polytope){
        Eigen::Vector3d box_max, box_min;
        box_min = pt;
        box_max = pt;
        box_min -= Vec3f(dis, dis, dis);
        box_max += Vec3f(dis, dis, dis);
        MatD4f planes;
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;
        bd(0, 3) = -box_max.x();
        bd(1, 3) = box_min.x();
        bd(2, 3) = -box_max.y();
        bd(3, 3) = box_min.y();
        bd(4, 3) = -box_max.z();
        bd(5, 3) = box_min.z();
        // 障碍物点云为空，直接返回一个方块
        // Ax + By + Cz + D = 0
        planes.resize(6, 4);
        planes.row(0) << 1, 0, 0, -box_max.x();
        planes.row(1) << 0, 1, 0, -box_max.y();
        planes.row(2) << 0, 0, 1, -box_max.z();
        planes.row(3) << -1, 0, 0, box_min.x();
        planes.row(4) << 0, -1, 0, box_min.y();
        planes.row(5) << 0, 0, -1, box_min.z();
        polytope.SetPlanes(planes);
        polytope.SetSeedLine(Line{pt, pt});
        return true;
    }

    bool CorridorGenerator::GeneratePolytopeFromLine(Line &line, Polytope &polytope) {
        Eigen::Vector3d box_max, box_min;
        vec_E<Vec3f> pc, pts{line.first, line.second};
        getSeedBBox(line.first, line.second, box_min, box_max);
        map_ptr_->boundBoxByLocalMap(box_min, box_max);
        map_ptr_->boxSearch(box_min, box_max, OCCUPIED, pc);
        box_min.z() += robot_r_;
        box_max.z() -= robot_r_;
        MatD4f planes;
        Eigen::Vector3d a = line.first, b = line.second;
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;
        bd(0, 3) = -box_max.x();
        bd(1, 3) = box_min.x();
        bd(2, 3) = -box_max.y();
        bd(3, 3) = box_min.y();
        bd(4, 3) = -box_max.z();
        bd(5, 3) = box_min.z();
        // 将vector放到Eigen里，准备开始分解
        if (pc.empty()) {
            // 障碍物点云为空，直接返回一个方块
            // Ax + By + Cz + D = 0g
            planes.resize(6, 4);
            planes.row(0) << 1, 0, 0, -box_max.x();
            planes.row(1) << 0, 1, 0, -box_max.y();
            planes.row(2) << 0, 0, 1, -box_max.z();
            planes.row(3) << -1, 0, 0, box_min.x();
            planes.row(4) << 0, -1, 0, box_min.y();
            planes.row(5) << 0, 0, -1, box_min.z();
            polytope.SetPlanes(planes);
            polytope.SetSeedLine(line);
            return true;
        }
        // save to latest pc
        latest_pc.insert(latest_pc.end(), pc.begin(), pc.end());
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc[0].data(), 3, pc.size());
        rog_map::TimeConsuming tc("emvp", false);
        RET_CODE success = ciri_->comvexDecomposition(bd, pp, a, b);
        double dt = tc.stop();
        if (success == SUCCESS) {
            ciri_cnt++;
            ciri_t += dt;
            ciri_->getPolytope(polytope);
            polytope.SetSeedLine(line);
            return true;
        } else {
            polytope.Reset();
            cout << YELLOW << "\t box_min = " << box_min.transpose() << RESET << endl;
            cout << YELLOW << "\t box_max =" << box_max.transpose() << RESET << endl;
            cout << YELLOW << "\t seed line =" << line.first.transpose() << " --> " << line.second.transpose()
                 << RESET << endl;

//            failed_traj_log << 889900 << endl;
//            failed_traj_log << bd << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << pp << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << a.transpose() << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << b.transpose() << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << robot_r_ << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << iris_iter_num_ << endl;
            return false;
        }

    }
}