    if ((cfg_.penna_pos > 0 && opt_vars.penalty_log(1) > 0.2) ||
        (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin) ||
        (cfg_.penna_acc > 0 && opt_vars.penalty_log(3) > cfg_.max_acc * cfg_.penna_margin) ||
        (cfg_.penna_omg > 0 && opt_vars.penalty_log(6) > cfg_.max_omg * cfg_.penna_margin) ||
        (cfg_.penna_thr > 0 && opt_vars.penalty_log(7) > cfg_.max_acc * cfg_.penna_margin)) {
        ret = -1;
        if (cfg_.print_optimizer_log) {
            cout << " -- [BaclOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
            cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
            cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
            cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
            cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
            cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
            cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
            cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
            cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
            cout << "\tTs: " << opt_vars.ts << endl;
        }
        ros_ptr_->warn(" -- [BackOpt] Opt failed, Omg or thr or Pos violation.");
    }


bool BackupTrajOpt::checkTrajMagnitudeBound(Trajectory &out_traj) {
    if (cfg_.penna_vel > 0 && out_traj.getMaxVelRate() > 1.2 * cfg_.max_vel) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max vel:\t" << out_traj.getMaxVelRate() << " m/s" << RESET
                  << std::endl;
        return false;
    }
    if (cfg_.penna_acc > 0 && out_traj.getMaxAccRate() > 1.2 * cfg_.max_acc) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max Acc:\t" << out_traj.getMaxAccRate() << " m/s" << RESET
                  << std::endl;
        return false;
    }
    return true;
}
