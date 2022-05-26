#include <plan_manage/planner_manager.h>
#include <memory>
#include <thread>
#include <utility>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner {

    // SECTION interfaces for setup and query

    EGOPlannerManager::EGOPlannerManager() = default;

    EGOPlannerManager::~EGOPlannerManager() = default;

    void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis) {
        /* read algorithm parameters */

        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
        nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
        nh.param("manager/drone_id", pp_.drone_id, -1);

        local_data_.traj_id_ = 0;
        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);

        bspline_optimizer_ = std::make_unique<BsplineOptimizer>();
        bspline_optimizer_->setParam(nh);
        bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
        bspline_optimizer_->a_star_.reset(new AStar);
        bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

        visualization_ = std::move(vis);
    }

    // !SECTION

    // SECTION rebond replanning

    bool EGOPlannerManager::reboundReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                          const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
                                          const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
                                          const bool flag_randomPolyTraj) {
        static int count = 0;
        printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id,
               count++);

        if ((start_pt - local_target_pt).norm() < 0.2) {
            cout << "Close to goal" << endl;
            continuous_failures_count_++;
            return false;
        }

        bspline_optimizer_->setLocalTargetPt(local_target_pt);

        ros::Time t_start = ros::Time::now();
        ros::Duration t_init, t_opt, t_refine;

        /*** STEP 1: INIT ***/
        double ts = (start_pt - local_target_pt).norm() > 0.1 ?
                    pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 :
                    pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
        // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        vector<Eigen::Vector3d> point_set, start_end_derivatives;
        static bool flag_first_call = true, flag_force_polynomial = false;
        bool flag_regenerate = false;
        do {
            point_set.clear();
            start_end_derivatives.clear();
            flag_regenerate = false;

            if (flag_first_call || flag_polyInit || flag_force_polynomial) {
                // Initial path generated from a min-snap traj by order.
                flag_first_call = false;
                flag_force_polynomial = false;
                initpath1(ts, flag_randomPolyTraj, start_pt, start_vel, start_acc, local_target_pt, local_target_vel, start_end_derivatives,
                          point_set);
            } else {
                // Initial path generated from previous trajectory.
                bool init_fail = false;
                flag_regenerate = initPathFromPreviousTrajectory(ts, init_fail, flag_force_polynomial,
                                                                 local_target_pt, local_target_vel, start_end_derivatives, point_set);
                if (init_fail) { return false; }
            }
        } while (flag_regenerate);

        Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

        vector<std::pair<int, int>> segments;
        segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

        t_init = ros::Time::now() - t_start;
        t_start = ros::Time::now();

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = false;
        vector<vector<Eigen::Vector3d>> vis_trajs;

        if (pp_.use_distinctive_trajs) {
            std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
            cout << "\033[1;33m" << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

            double final_cost, min_cost = 999999.0;

            for (int i = trajs.size() - 1; i >= 0; i--) {
                if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts)) {
                    cout << "traj " << trajs.size() - i << " success." << endl;
                    flag_step_1_success = true;
                    if (final_cost < min_cost) {
                        min_cost = final_cost;
                        ctrl_pts = ctrl_pts_temp;
                    }

                    // visualization
                    point_set.clear();
                    for (int j = 0; j < ctrl_pts_temp.cols(); j++) {
                        point_set.emplace_back(ctrl_pts_temp.col(j));
                    }
                    vis_trajs.push_back(point_set);
                } else {
                    cout << "traj " << trajs.size() - i << " failed." << endl;
                }
            }

            t_opt = ros::Time::now() - t_start;

            // This visualization will take up several milliseconds.
            visualization_->displayMultiInitPathList(vis_trajs, 0.2);
        } else {
            flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
            t_opt = ros::Time::now() - t_start;
            visualization_->displayInitPathList(point_set, 0.2, 0);
        }

        cout << "plan_success=" << flag_step_1_success << endl;
        if (!flag_step_1_success) {
            visualization_->displayOptimalList(ctrl_pts, 0);
            continuous_failures_count_++;
            return false;
        }

        t_start = ros::Time::now();

        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
        if (pp_.drone_id <= 0) {

            double ratio;
            bool flag_step_2_success = true;
            if (!pos.checkFeasibility(ratio, false)) {
                cout << "Need to reallocate time." << endl;

                Eigen::MatrixXd optimal_control_points;
                flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
                if (flag_step_2_success)
                    pos = UniformBspline(optimal_control_points, 3, ts);
            }

            if (!flag_step_2_success) {
                printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. "
                       "But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
                continuous_failures_count_++;
                return false;
            }
        } else {
            static bool print_once = true;
            if (print_once) {
                print_once = false;
                ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
            }
        }

        t_refine = ros::Time::now() - t_start;

        // save planned results
        updateTrajInfo(pos, ros::Time::now());

        static double sum_time = 0;
        static int count_success = 0;
        sum_time += (t_init + t_opt + t_refine).toSec();
        count_success++;
        cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:"
             << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success
             << endl;

        // success. YoY
        continuous_failures_count_ = 0;
        return true;
    }

    bool EGOPlannerManager::initpath1(double &ts, const bool flag_randomPolyTraj,
                                      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
                                      vector<Eigen::Vector3d> &start_end_derivatives, vector<Eigen::Vector3d> &point_set
    ) {
        // Initial path generated from a min-snap traj by order.

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ?
                      sqrt(dist / pp_.max_acc_) :
                      (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj) {
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt,
                                                           local_target_vel, Eigen::Vector3d::Zero(), time);
        } else {
            Eigen::Vector3d horizontal_dir = ((start_pt - local_target_pt).cross(
                    Eigen::Vector3d(0, 0, 1))).normalized();
            Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizontal_dir)).normalized();
            double rand1 = ((double) rand()) / RAND_MAX - 0.5;
            double rand2 = ((double) rand()) / RAND_MAX - 0.5;
            double magic = -0.978 / (continuous_failures_count_ + 0.989) + 0.989;
            double length = (start_pt - local_target_pt).norm();
            Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                 rand1 * length * horizontal_dir * 0.8 * magic +
                                                 rand2 * length * vertical_dir * 0.4 * magic;
            Eigen::MatrixXd pos(3, 3);
            pos.col(0) = start_pt;
            pos.col(1) = random_inserted_pt;
            pos.col(2) = local_target_pt;
            Eigen::VectorXd t(2);
            t(0) = t(1) = time / 2;
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do {
            ts /= 1.5;
            point_set.clear();
            flag_too_far = false;
            Eigen::Vector3d last_pt = gl_traj.evaluate(0);
            for (t = 0; t < time; t += ts) {
                Eigen::Vector3d pt = gl_traj.evaluate(t);
                if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) {
                    flag_too_far = true;
                    break;
                }
                last_pt = pt;
                point_set.push_back(pt);
            }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.emplace_back(gl_traj.evaluateVel(0));
        start_end_derivatives.emplace_back(local_target_vel);
        start_end_derivatives.emplace_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.emplace_back(gl_traj.evaluateAcc(t));
    }

    bool EGOPlannerManager::initPathFromPreviousTrajectory(const double ts, bool &init_fail, bool &flag_force_polynomial,
                                                           const Eigen::Vector3d local_target_pt, const Eigen::Vector3d local_target_vel,
                                                           vector<Eigen::Vector3d> &start_end_derivatives, vector<Eigen::Vector3d> &point_set) {
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);

        double t, t_cur = (ros::Time::now() - local_data_.start_time_).toSec();
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts) {
            segment_point.emplace_back(local_data_.position_traj_.evaluateDeBoorT(t));
            if (t > t_cur) {
                double cur_length = (segment_point.back() - segment_point[segment_point.size() - 2]).norm();
                pseudo_arc_length.push_back(cur_length + pseudo_arc_length.back());
            }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts) {
            PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
                    local_data_.position_traj_.evaluateDeBoorT(t),
                    local_data_.velocity_traj_.evaluateDeBoorT(t),
                    local_data_.acceleration_traj_.evaluateDeBoorT(t),
                    local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

            for (t = ts; t < poly_time; t += ts) {
                if (!pseudo_arc_length.empty()) {
                    segment_point.push_back(gl_traj.evaluate(t));
                    double cur_length = (segment_point.back() - segment_point[segment_point.size() - 2]).norm();
                    pseudo_arc_length.push_back(cur_length + pseudo_arc_length.back());
                } else {
                    ROS_ERROR("pseudo_arc_length is empty, return!");
                    continuous_failures_count_++;
                    init_fail = true;
                    return false;
                }
            }
        }

        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        do {
            cps_dist /= 1.5;
            point_set.clear();
            double sample_length = 0;
            size_t id = 0;
            while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back()) {
                if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1]) {
                    double ratio1 = (sample_length - pseudo_arc_length[id]) /
                                    (pseudo_arc_length[id + 1] - pseudo_arc_length[id]);
                    double ratio2 = (pseudo_arc_length[id + 1] - sample_length) /
                                    (pseudo_arc_length[id + 1] - pseudo_arc_length[id]);
                    point_set.emplace_back(ratio1 * segment_point[id + 1] + ratio2 * segment_point[id]);
                    sample_length += cps_dist;
                } else
                    id++;
            }
            point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.emplace_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.emplace_back(local_target_vel);
        start_end_derivatives.emplace_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.emplace_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) {
            // The initial path is un normally too long!
            flag_force_polynomial = true;
            return true;
        }
        return false;
    }


    bool EGOPlannerManager::EmergencyStop(const Eigen::Vector3d &stop_pos) {
        Eigen::MatrixXd control_points(3, 6);
        for (int i = 0; i < 6; i++) {
            control_points.col(i) = stop_pos;
        }

        updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

        return true;
    }

    bool EGOPlannerManager::checkCollision(int drone_id) {
        if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
            return false;

        double my_traj_start_time = local_data_.start_time_.toSec();
        double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

        double t_start = max(my_traj_start_time, other_traj_start_time);
        double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3,
                           other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

        for (double t = t_start; t < t_end; t += 0.03) {
            if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) -
                 swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() <
                bspline_optimizer_->getSwarmClearance()) {
                return true;
            }
        }

        return false;
    }

    bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel,
                                           const Eigen::Vector3d &end_acc) {

        // generate global reference trajectory
        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);
        points.push_back(end_pos);

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        const double dist_thresh = 4.0;

        for (size_t i = 0; i < points.size() - 1; ++i) {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh) {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j) {
                    Eigen::Vector3d inter_pt =
                            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        // write position matrix
        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i) {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc,
                                                           time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool
    EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                      double &ts, Eigen::MatrixXd &optimal_control_points) {
        double t_inc;

        Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        traj = UniformBspline(ctrl_pts, 3, ts);

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
        bspline_optimizer_->ref_pts_.clear();
        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_->ref_pts_.emplace_back(traj.evaluateDeBoorT(t));

        bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

        return success;
    }

    void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now) {
        local_data_.start_time_ = time_now;
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative,
                                           double ratio,
                                           Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc) {
        double time_origin = bspline.getTimeSum();
        Eigen::Index seg_num = bspline.getControlPoint().cols() - 3;

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt) {
            point_set.emplace_back(bspline.evaluateDeBoorT(time));
        }
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

} // namespace ego_planner