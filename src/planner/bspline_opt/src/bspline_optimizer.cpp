#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace ego_planner {

    void BsplineOptimizer::setParam(ros::NodeHandle &nh) {
        nh.param("optimization/lambda_smooth", lambda1_, -1.0);
        nh.param("optimization/lambda_collision", lambda2_, -1.0);
        nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
        nh.param("optimization/lambda_fitness", lambda4_, -1.0);

        nh.param("optimization/dist0", dist0_, -1.0);
        nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
        nh.param("optimization/max_vel", max_vel_, -1.0);
        nh.param("optimization/max_acc", max_acc_, -1.0);

        nh.param("optimization/order", order_, 3);
    }

    void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map) {
        this->grid_map_ = map;
    }

    void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr &mov_obj) {
        this->grid_map_ = map;
        this->moving_objs_ = mov_obj;
    }

    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points) {
        cps_.points = points;
    }

    void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

    void BsplineOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

    void BsplineOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

    std::vector <ControlPoints> BsplineOptimizer::distinctiveTrajs(vector <std::pair<int, int>> segments) {
        if (segments.empty()) {// will be invoked again later.
            std::vector <ControlPoints> oneSeg;
            oneSeg.push_back(cps_);
            return oneSeg;
        }

        int seg_upbound = std::min((int) segments.size(), 3);
        const double RESOLUTION = grid_map_->getResolution();
        const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1);

        // Step 1. Find the opposite vectors and base points for every segment.
        vector <ControlPoints> origin_cps_segments, reversed_cps_segments;
        vector <std::pair<int, int>> valid_segments;
        for (const auto seg: segments) {
            ControlPoints origin_cps_segment;
            cps_.segment(origin_cps_segment, seg.first, seg.second);
            bool valid = origin_cps_segment.size >= 0;
            ControlPoints reversed_cps_segment;
            if (origin_cps_segment.size > 1) {
                bool error = false;
                valid = reverseCpsBasePoint(RESOLUTION, CTRL_PT_DIST, origin_cps_segment, reversed_cps_segment, error);
                if (error) { return {}; }
            } else if (origin_cps_segment.size == 1) {
                valid = reverseBasePointForSingleControlPoint(RESOLUTION, CTRL_PT_DIST, origin_cps_segment, reversed_cps_segment);
            }
            if (valid) {
                origin_cps_segments.emplace_back(origin_cps_segment);
                reversed_cps_segments.emplace_back(reversed_cps_segment);
                valid_segments.emplace_back(seg);
            }
        }
        segments = move(valid_segments);
        seg_upbound = segments.size();

        // Step 2. Assemble each segment to make up the new control point sequence.
        if (segments.empty()) { // After the erase operation above, segment legth will decrease to 0 again.
            std::vector <ControlPoints> oneSeg;
            oneSeg.push_back(cps_);
            return oneSeg;
        }

        std::vector <ControlPoints> control_pts_buf;
        int max_traj_nums = (1 << seg_upbound);
        for (int traj_id = 0; traj_id < max_traj_nums; ++traj_id) {
            ControlPoints cpsOneSample;
            cpsOneSample.resize(cps_.size);
            cpsOneSample.clearance = cps_.clearance;
            int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
            while (cp_id < cps_.size) {
                if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second) {
                    // 我们只考虑前三个障碍物，seg_id >= seg_upbound后面的障碍物就不考虑了
                    // cp_id < segments[seg_id].first || cp_id > segments[seg_id].second说明当前控制点没有在障碍物里面
                    cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
                    cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
                    cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
                } else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second) {
                    // 现在控制点在前三个障碍物里面
                    if (!((traj_id >> seg_id) & 1)) { //状态压缩
                        //按照状态压缩应该用原来的控制点
                        cpsOneSample.points.col(cp_id) = origin_cps_segments[seg_id].points.col(cp_of_seg_id);
                        cpsOneSample.base_point[cp_id] = origin_cps_segments[seg_id].base_point[cp_of_seg_id];
                        cpsOneSample.direction[cp_id] = origin_cps_segments[seg_id].direction[cp_of_seg_id];
                    } else if (reversed_cps_segments[seg_id].size) {
                        //按照状态压缩，应该用另一边的控制点。如果找到了障碍物另一边的控制点，那就用另一边的控制点。
                        cpsOneSample.points.col(cp_id) = reversed_cps_segments[seg_id].points.col(cp_of_seg_id);
                        cpsOneSample.base_point[cp_id] = reversed_cps_segments[seg_id].base_point[cp_of_seg_id];
                        cpsOneSample.direction[cp_id] = reversed_cps_segments[seg_id].direction[cp_of_seg_id];
                    } else {
                        goto abandon_this_trajectory;
                    }
                    cp_of_seg_id++;

                    if (cp_id == segments[seg_id].second) {
                        cp_of_seg_id = 0;
                        seg_id++;
                    }
                } else {
                    ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, "
                              "segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                              cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first,
                              segments[seg_id].second);
                }
                cp_id++;
            }

            control_pts_buf.emplace_back(cpsOneSample);
            abandon_this_trajectory:;
        }
        return control_pts_buf;
    }

    // 与reverseCpsBasePoint类似，不再单独写注释
    bool BsplineOptimizer::reverseBasePointForSingleControlPoint(const double RESOLUTION, const double CTRL_PT_DIST,
                                                                 const ControlPoints &origin_cps, ControlPoints &reversed_cps) {
        reversed_cps = origin_cps;//这一步的目的是为base_point和direction分配内存。
        Eigen::Vector3d base_vec_reverse = -origin_cps.direction[0][0];
        Eigen::Vector3d base_pt_reverse = origin_cps.points.col(0) +
                                          base_vec_reverse * (origin_cps.base_point[0][0] - origin_cps.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) {
            // Search outward.
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION) {
                Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
                if (!grid_map_->getInflateOccupancy(base_pt_temp)) {
                    reversed_cps.base_point[0][0] = base_pt_temp;
                    reversed_cps.direction[0][0] = base_vec_reverse;
                    break;
                }
            }
            if (l > l_upbound) {
                ROS_WARN("Can't find the new base points at the opposite within the threshold");
                return false;
            }
        } else if ((base_pt_reverse - origin_cps.points.col(0)).norm() >= RESOLUTION) {
            // Unnecessary to search.
            reversed_cps.base_point[0][0] = base_pt_reverse;
            reversed_cps.direction[0][0] = base_vec_reverse;
        } else {
            ROS_WARN("base_point and control point are too close!, 2");
            return false;
        }
        return true;
    }

    /**
     * @brief 原始控制点origin_cps对应的基点都在障碍物的一侧，我们希望能够在障碍物的另一侧找到一组基点和方向，作为reversed_cps。
     * @return 没找到的话返回false
     * @param error 原始控制点数据有问题的话error=true
     * */
    bool BsplineOptimizer::reverseCpsBasePoint(const double RESOLUTION, const double CTRL_PT_DIST,
                                               const ControlPoints &origin_cps, ControlPoints &reversed_cps, bool &error) {
        reversed_cps = origin_cps;//这一步的目的是为base_point和direction分配内存。
        error = false;
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        // 在原始控制点中插值，找到原始轨迹进入障碍物的点occ_start_pt
        for (int j = 0; j < origin_cps.size - 1; j++) {
            double step_size = RESOLUTION / (origin_cps.points.col(j) - origin_cps.points.col(j + 1)).norm() / 2;
            for (double a = 1; a > 0; a -= step_size) {
                Eigen::Vector3d pt(a * origin_cps.points.col(j) + (1 - a) * origin_cps.points.col(j + 1));
                if (grid_map_->getInflateOccupancy(pt)) {
                    occ_start_id = j;
                    occ_start_pt = pt;
                    goto exit_multi_loop1;
                }
            }
        }
        exit_multi_loop1:;

        // 在原始控制点中插值，找到原始轨迹离开障碍物的点occ_end_pt
        for (int j = origin_cps.size - 1; j >= 1; j--) {
            double step_size = RESOLUTION / (origin_cps.points.col(j) - origin_cps.points.col(j - 1)).norm();
            for (double a = 1; a > 0; a -= step_size) {
                Eigen::Vector3d pt(a * origin_cps.points.col(j) + (1 - a) * origin_cps.points.col(j - 1));
                if (grid_map_->getInflateOccupancy(pt)) {
                    occ_end_id = j;
                    occ_end_pt = pt;
                    goto exit_multi_loop2;
                }
            }
        }
        exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1) {
            // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
            ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);
            return false;
        }

        /*******************************************************************************
         * Reverse the vector and find new base points from occ_start_id to occ_end_id *
         *******************************************************************************/
        for (int j = occ_start_id; j <= occ_end_id; j++) {
            Eigen::Vector3d base_pt_reverse, base_vec_reverse;
            if (origin_cps.base_point[j].size() != 1) {
                ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");
                error = true;
                return false;
            }

            base_vec_reverse = -origin_cps.direction[j][0];

            // The start and the end case must get taken special care of.
            if (j == occ_start_id) {
                base_pt_reverse = occ_start_pt;
            } else if (j == occ_end_id) {
                base_pt_reverse = occ_end_pt;
            } else {
                base_pt_reverse = origin_cps.points.col(j) +
                                  base_vec_reverse * (origin_cps.base_point[j][0] - origin_cps.points.col(j)).norm();
            }

            // 以原始基点为出发点，沿base_vec反向延伸，每次延伸RESOLUTION，直到走出障碍物。
            // 如果延伸了5 * CTRL_PT_DIST还没有走出障碍物，返回false。
            if (grid_map_->getInflateOccupancy(base_pt_reverse)) {
                // Search outward.
                double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
                double l = RESOLUTION;
                for (; l <= l_upbound; l += RESOLUTION) {
                    Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
                    if (!grid_map_->getInflateOccupancy(base_pt_temp)) {
                        reversed_cps.base_point[j][0] = base_pt_temp;
                        reversed_cps.direction[j][0] = base_vec_reverse;
                        break;
                    }
                }
                if (l > l_upbound) {
                    ROS_WARN("Can't find the new base points at the opposite within the threshold, j=%d", j);
                    return false;
                }
            } else if ((base_pt_reverse - origin_cps.points.col(j)).norm() >= RESOLUTION) {
                // Unnecessary to search.
                reversed_cps.base_point[j][0] = base_pt_reverse;
                reversed_cps.direction[j][0] = base_vec_reverse;
            } else {
                ROS_WARN("base_point and control point are too close!");
                cout << "base_point=" << origin_cps.base_point[j][0].transpose() << " control point="
                     << origin_cps.points.col(j).transpose() << endl;
                return false;
            }
        }

        /*****************************************************************************************************
         * Assign the base points to control points within [0, occ_start_id) and (occ_end_id, origin_cps.size()-1] *
         *****************************************************************************************************/
        if (reversed_cps.size) {
            for (int j = occ_start_id - 1; j >= 0; j--) {
                reversed_cps.base_point[j][0] = reversed_cps.base_point[occ_start_id][0];
                reversed_cps.direction[j][0] = reversed_cps.direction[occ_start_id][0];
            }
            for (int j = occ_end_id + 1; j < reversed_cps.size; j++) {
                reversed_cps.base_point[j][0] = reversed_cps.base_point[occ_end_id][0];
                reversed_cps.direction[j][0] = reversed_cps.direction[occ_end_id][0];
            }
        }
        return true;
    }

    std::vector <std::pair<int, int>>
    BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/) {
        if (flag_first_init) {
            cps_.clearance = dist0_;
            cps_.resize(init_points.cols());
            cps_.points = init_points;
        }

        /*** Segment the initial trajectory according to obstacles ***/
        constexpr int ENOUGH_INTERVAL = 2;
        double step_size = grid_map_->getResolution() / 1.5 /
                           ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1));
        int in_id = -1, out_id = -1;
        vector <std::pair<int, int>> segment_ids;
        int same_occ_state_times = ENOUGH_INTERVAL + 1;
        bool occ, last_occ = false;
        bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
        int i_end = (int) init_points.cols() - order_ -
                    ((int) init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
        for (int i = order_; i <= i_end; ++i) {
            for (double a = 1.0; a > 0.0; a -= step_size) {
                occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
                if (occ && !last_occ) {
                    if (same_occ_state_times > ENOUGH_INTERVAL || i == order_) {
                        in_id = i - 1;
                        flag_got_start = true;
                    }
                    same_occ_state_times = 0;
                    flag_got_end_maybe = false; // terminate in advance
                } else if (!occ && last_occ) {
                    out_id = i;
                    flag_got_end_maybe = true;
                    same_occ_state_times = 0;
                } else {
                    ++same_occ_state_times;
                }

                if (flag_got_end_maybe &&
                    (same_occ_state_times > ENOUGH_INTERVAL || (i == (int) init_points.cols() - order_))) {
                    flag_got_end_maybe = false;
                    flag_got_end = true;
                }

                last_occ = occ;

                if (flag_got_start && flag_got_end) {
                    flag_got_start = false;
                    flag_got_end = false;
                    segment_ids.emplace_back(std::pair<int, int>(in_id, out_id));
                }
            }
        }

        // return in advance
        if (segment_ids.empty()) {
            vector <std::pair<int, int>> blank_ret;
            return blank_ret;
        }

        /*** calculate bounds ***/
        Eigen::Index id_low_bound, id_up_bound;
        vector <std::pair<int, int>> bounds(segment_ids.size());
        for (size_t i = 0; i < segment_ids.size(); i++) {
            if (i == segment_ids.size() - 1) {
                id_up_bound = init_points.cols() - order_ - 1;
            } else {
                id_up_bound = (int) (((float) (segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);
            }

            if (i == 0) {
                id_low_bound = order_;
            } else {
                id_low_bound = (int) (((float) (segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2);
            }

            bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
        }

        /*** Adjust segment length ***/
        for (size_t i = 1; i < segment_ids.size(); i++) { // Avoid overlap
            if (segment_ids[i - 1].second >= segment_ids[i].first) {
                double middle = (double) (segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
                segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
                segment_ids[i].first = static_cast<int>(middle + 1.1);
            }
        }

        // Used for return
        vector <std::pair<int, int>> final_segment_ids;

        /*** Assign data to each segment ***/
        for (auto &segment_id: segment_ids) {
            bool a_star_success, base_point_success;
            getBasePointAndDirectionForSegment(segment_id.first, segment_id.second, cps_, a_star_success, base_point_success);
            if (!a_star_success) {
                return {};
            }
            if (base_point_success) {
                final_segment_ids.emplace_back(segment_id);
            }
        }

        return final_segment_ids;
    }

    int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx,
                                    const double xnorm, const double gnorm, const double step, int n, int k, int ls) {
        auto *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
    }

    double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n) {
        auto *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

        double cost;
        opt->combineCostRebound(x, grad, cost, n);

        opt->iter_num_ += 1;
        return cost;
    }

    double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n) {
        auto *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

        double cost;
        opt->combineCostRefine(x, grad, cost, n);

        opt->iter_num_ += 1;
        return cost;
    }

    void BsplineOptimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient) {
        cost = 0.0;
        // Only check the first 2/3 points
        Eigen::Index end_idx = q.cols() - order_ - (double) (q.cols() - 2 * order_) * 1.0 / 3.0;
        const double CLEARANCE = swarm_clearance_ * 2;
        double t_now = ros::Time::now().toSec();
        constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

        for (int i = order_; i < end_idx; i++) {
            double glb_time = t_now + ((double) (order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

            for (size_t id = 0; id < swarm_trajs_->size(); id++) {
                if ((swarm_trajs_->at(id).drone_id != (int) id) || swarm_trajs_->at(id).drone_id == drone_id_) {
                    continue;
                }

                double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.toSec();
                if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ - 0.1) {
                    /* def cost=(c-sqrt([Q-O]'D[Q-O]))^2, D=[1/b^2,0,0;0,1/b^2,0;0,0,1/a^2] */
                    Eigen::Vector3d swarm_prid = swarm_trajs_->at(id).position_traj_.evaluateDeBoorT(
                            glb_time - traj_i_satrt_time);
                    Eigen::Vector3d dist_vec = cps_.points.col(i) - swarm_prid;
                    double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 +
                                             (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
                    double dist_err = CLEARANCE - ellip_dist;

                    Eigen::Vector3d dist_grad = cps_.points.col(i) - swarm_prid;
                    Eigen::Vector3d Coeff;
                    Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
                    Coeff(1) = Coeff(0);
                    Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;

                    if (dist_err < 0) {
                        /* do nothing */
                    } else {
                        cost += pow(dist_err, 2);
                        gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
                    }

                    if (min_ellip_dist_ > dist_err) {
                        min_ellip_dist_ = dist_err;
                    }
                }
            }
        }
    }

    void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                   Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost) {
        cost = 0.0;
        Eigen::Index end_idx = q.cols() - order_;
        double demarcation = cps_.clearance;
        double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

        force_stop_type_ = DONT_STOP;
        if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) {
            // 0.1 is an experimental value that indicates the trajectory is smooth enough.
            check_collision_and_rebound();
        }

        /*** calculate distance cost and gradient ***/
        for (auto i = order_; i < end_idx; ++i) {
            for (size_t j = 0; j < cps_.direction[i].size(); ++j) {
                double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
                double dist_err = cps_.clearance - dist;
                Eigen::Vector3d dist_grad = cps_.direction[i][j];

                if (dist_err < 0) {
                    /* do nothing */
                } else if (dist_err < demarcation) {
                    cost += pow(dist_err, 3);
                    gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
                } else {
                    cost += a * dist_err * dist_err + b * dist_err + c;
                    gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
                }
            }
        }
    }

    void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient) const {

        cost = 0.0;

        Eigen::Index end_idx = q.cols() - order_;

        // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
        double a2 = 25, b2 = 1;
        for (auto i = order_ - 1; i < end_idx + 1; ++i) {
            Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
            Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

            double xdotv = x.dot(v);
            Eigen::Vector3d xcrossv = x.cross(v);

            double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
            cost += f;

            Eigen::Matrix3d m;
            m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
            Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

            gradient.col(i - 1) += df_dx / 6;
            gradient.col(i) += 4 * df_dx / 6;
            gradient.col(i + 1) += df_dx / 6;
        }
    }

    void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                              Eigen::MatrixXd &gradient, bool falg_use_jerk) {
        cost = 0.0;

        if (falg_use_jerk) {
            Eigen::Vector3d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++) {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        } else {
            Eigen::Vector3d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++) {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }

    void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient) {
        cost = 0.0;

        // zero cost and gradient in hard constraints
        Eigen::Vector3d q_3, q_2, q_1, dq;
        q_3 = q.col(q.cols() - 3);
        q_2 = q.col(q.cols() - 2);
        q_1 = q.col(q.cols() - 1);

        dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
        cost += dq.squaredNorm();

        gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
        gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
        gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
    }

    void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                               Eigen::MatrixXd &gradient) const {
        cost = 0.0;
        /* abbreviation */
        double ts, /*vm2, am2, */ ts_inv2;

        ts = bspline_interval_;
        ts_inv2 = 1 / ts / ts;

        /* velocity feasibility */
        for (int i = 0; i < q.cols() - 1; i++) {
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
            for (int j = 0; j < 3; j++) {
                if (vi(j) > max_vel_) {
                    // multiply ts_inv3 to make vel and acc has similar magnitude
                    cost += pow(vi(j) - max_vel_, 2) * ts_inv2;
                    gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
                } else if (vi(j) < -max_vel_) {
                    cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

                    gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
                } else {
                    /* code */
                }
            }
        }

        /* acceleration feasibility */
        for (int i = 0; i < q.cols() - 2; i++) {
            Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
            for (int j = 0; j < 3; j++) {
                if (ai(j) > max_acc_) {
                    cost += pow(ai(j) - max_acc_, 2);

                    gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
                    gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
                    gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
                } else if (ai(j) < -max_acc_) {
                    cost += pow(ai(j) + max_acc_, 2);

                    gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
                    gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
                    gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
                }
            }
        }
    }

    void BsplineOptimizer::getBasePointAndDirectionForSegment(const int start_id, const int end_id,
                                                              ControlPoints &cps, bool &a_star_success, bool &base_point_success) {
        for (int j = start_id; j <= end_id; ++j)
            cps.flag_temp[j] = false;
        vector <Eigen::Vector3d> a_star_path;
        if (a_star_->AstarSearch(0.1, cps.points.col(start_id), cps.points.col(end_id))) {
            a_star_path = a_star_->getPath();
            a_star_success = true;
        } else {
            a_star_success = false;
            return;
        }

        // step 2
        int got_intersection_id = -1;
        for (int j = start_id + 1; j < end_id; ++j) {
            Eigen::Vector3d ctrl_pts_law(cps.points.col(j + 1) - cps.points.col(j - 1)), intersection_point;
            // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            int Astar_id = a_star_path.size() / 2, last_Astar_id;
            double val = (a_star_path[Astar_id] - cps.points.col(j)).dot(ctrl_pts_law), last_val = val;
            while (Astar_id >= 0 && Astar_id < (int) a_star_path.size()) {
                last_Astar_id = Astar_id;
                last_val = val;

                if (val >= 0)
                    --Astar_id;
                else
                    ++Astar_id;

                val = (a_star_path[Astar_id] - cps.points.col(j)).dot(ctrl_pts_law);

                if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) {// val = last_val = 0.0 is not allowed
                    double ratio = ctrl_pts_law.dot(a_star_path[Astar_id] - cps.points.col(j)) /
                                   ctrl_pts_law.dot(a_star_path[Astar_id] - a_star_path[last_Astar_id]);
                    intersection_point =
                            (1 - ratio) * a_star_path[Astar_id] + ratio * a_star_path[last_Astar_id];
                    got_intersection_id = j;
                    break;
                }
            }

            if (got_intersection_id >= 0) {
                double length = (intersection_point - cps.points.col(j)).norm();
                if (length > 1e-5) {
                    cps.flag_temp[j] = true;
                    for (double a = length; a >= 0.0; a -= grid_map_->getResolution()) {
                        Eigen::Vector3d line_point = (a / length) * intersection_point + (1 - a / length) * cps.points.col(j);
                        bool occ = grid_map_->getInflateOccupancy(line_point);

                        if (occ || a < grid_map_->getResolution()) {
                            if (occ) { a += grid_map_->getResolution(); }
                            cps.base_point[j].emplace_back(line_point);
                            cps.direction[j].emplace_back((intersection_point - cps.points.col(j)).normalized());
                            break;
                        }
                    }
                } else {
                    got_intersection_id = -1;
                }
            }
        }

        /**
         * Corner case: the segment length is too short,
         * the control points may outside the A* path,
         * leading to opposite gradient direction.
         * So I have to take special care of it */
        if (end_id - start_id == 1) {
            Eigen::Vector3d ctrl_pts_law(cps.points.col(end_id) - cps.points.col(start_id)), intersection_point;
            Eigen::Vector3d middle_point = (cps.points.col(end_id) + cps.points.col(start_id)) / 2;
            // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            int Astar_id = a_star_path.size() / 2, last_Astar_id;
            double val = (a_star_path[Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
            while (Astar_id >= 0 && Astar_id < (int) a_star_path.size()) {
                last_Astar_id = Astar_id;
                last_val = val;

                if (val >= 0)
                    --Astar_id;
                else
                    ++Astar_id;

                val = (a_star_path[Astar_id] - middle_point).dot(ctrl_pts_law);

                if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) {
                    // val = last_val = 0.0 is not allowed
                    double ratio = ctrl_pts_law.dot(a_star_path[Astar_id] - middle_point) /
                                   ctrl_pts_law.dot(a_star_path[Astar_id] - a_star_path[last_Astar_id]);
                    intersection_point = (1 - ratio) * a_star_path[Astar_id] + ratio * a_star_path[last_Astar_id];

                    if ((intersection_point - middle_point).norm() > 0.01) {// 1cm.
                        cps.flag_temp[start_id] = true;
                        cps.base_point[start_id].push_back(cps.points.col(start_id));
                        cps.direction[start_id].push_back(
                                (intersection_point - middle_point).normalized());

                        got_intersection_id = start_id;
                    }
                    break;
                }
            }
        }

        //step 3
        if (got_intersection_id >= 0) {
            for (int j = got_intersection_id + 1; j <= end_id; ++j)
                if (!cps.flag_temp[j]) {
                    cps.base_point[j].emplace_back(cps.base_point[j - 1].back());
                    cps.direction[j].emplace_back(cps.direction[j - 1].back());
                }

            for (int j = got_intersection_id - 1; j >= start_id; --j)
                if (!cps.flag_temp[j]) {
                    cps.base_point[j].emplace_back(cps.base_point[j + 1].back());
                    cps.direction[j].emplace_back(cps.direction[j + 1].back());
                }
            base_point_success = true;
        } else {
            base_point_success = false;
        }
    }

    bool BsplineOptimizer::check_collision_and_rebound() {
        int end_idx = cps_.size - order_;

        /*** Check and segment the initial trajectory according to obstacles ***/
        vector <std::pair<int, int>> segment_ids;
        bool flag_new_obs_valid = false;
        int i_end = end_idx - (end_idx - order_) / 3;
        for (int i = order_ - 1; i <= i_end; ++i) {
            bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));
            /*** check if the new collision will be valid ***/
            if (occ) {
                for (size_t k = 0; k < cps_.direction[i].size(); ++k) {
                    if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < grid_map_->getResolution()) {
                        // current point is outside all the collision_points.
                        occ = false; // Not really takes effect, just for better human understanding.
                        break;
                    }
                }
            }

            if (occ) {
                flag_new_obs_valid = true;
                int in_id = -1;
                for (in_id = i - 1; in_id >= 0; --in_id) {
                    occ = grid_map_->getInflateOccupancy(cps_.points.col(in_id));
                    if (!occ) { break; }
                }
                if (in_id < 0) {// fail to get the obs free point
                    ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
                    in_id = 0;
                }

                int out_id = -1;
                for (out_id = i + 1; out_id < cps_.size; ++out_id) {
                    occ = grid_map_->getInflateOccupancy(cps_.points.col(out_id));
                    if (!occ) { break; }
                }
                if (out_id >= cps_.size) {// fail to get the obs free point
                    ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");
                    force_stop_type_ = STOP_FOR_ERROR;
                    return false;
                }

                i = out_id + 1;
                segment_ids.emplace_back(in_id, out_id);
            }
        }

        if (!flag_new_obs_valid) { return false; }

        for (size_t i = 1; i < segment_ids.size(); i++) { // Avoid overlap
            if (segment_ids[i - 1].second >= segment_ids[i].first) {
                double middle = (double) (segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
                segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
                segment_ids[i].first = static_cast<int>(middle + 1.1);
            }
        }

        /*** Assign parameters to each segment ***/
        for (int i = 0; i < segment_ids.size(); ++i) {
            bool a_star_success, base_point_success;
            getBasePointAndDirectionForSegment(segment_ids[i].first, segment_ids[i].second, cps_, a_star_success, base_point_success);
            if (!a_star_success) {
                segment_ids.erase(segment_ids.begin() + i);
                i--;
            }
        }

        force_stop_type_ = STOP_FOR_REBOUND;
        return true;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts) {
        setBsplineInterval(ts);

        double final_cost;
        bool flag_success = rebound_optimize(final_cost);

        optimal_points = cps_.points;

        return flag_success;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost,
                                                      const ControlPoints &control_points, double ts) {
        setBsplineInterval(ts);

        cps_ = control_points;

        bool flag_success = rebound_optimize(final_cost);

        optimal_points = cps_.points;

        return flag_success;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts,
                                                     Eigen::MatrixXd &optimal_points) {

        setControlPoints(init_points);
        setBsplineInterval(ts);

        bool flag_success = refine_optimize();

        optimal_points = cps_.points;

        return flag_success;
    }

    bool BsplineOptimizer::rebound_optimize(double &final_cost) {
        iter_num_ = 0;
        int start_id = order_;
        int end_id = this->cps_.size;
        variable_num_ = 3 * (end_id - start_id);

        ros::Time t0 = ros::Time::now(), t1, t2;
        int restart_nums = 0, rebound_times = 0;;
        bool flag_force_return, flag_occ, success, ellip_flag;
        new_lambda2_ = lambda2_;
        constexpr int MAX_RESART_NUMS_SET = 3;
        do {
            /* ---------- prepare ---------- */
            min_ellip_dist_ = INIT_min_ellip_dist_;
            iter_num_ = 0;
            flag_force_return = false;
            flag_occ = false;
            success = false;

            double q[variable_num_];
            memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

            lbfgs::lbfgs_parameter_t lbfgs_params{};
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.01;

            /* ---------- optimize ---------- */
            t1 = ros::Time::now();
            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound,
                                               nullptr, BsplineOptimizer::earlyExit, this, &lbfgs_params);
            t2 = ros::Time::now();
            double time_ms = (t2 - t1).toSec() * 1000;
            double total_time_ms = (t2 - t0).toSec() * 1000;

            /* ---------- success temporary, check collision again ---------- */
            ellip_flag = (min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_);
            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP) {
                flag_force_return = false;

                /*** collision check, phase 1 ***/
                if (ellip_flag) {
                    success = false;
                    restart_nums++;
                    initControlPoints(cps_.points, false);
                    new_lambda2_ *= 2;
                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);
                    continue;
                }

                /*** collision check, phase 2 ***/
                UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
                double tm, tmp;
                traj.getTimeSpan(tm, tmp);
                double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() /
                                              grid_map_->getResolution());
                for (double t = tm; t < tmp * 2 / 3; t += t_step) {
                    // Only check the closest 2/3 partition of the whole trajectory.
                    flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
                    if (flag_occ) {
                        if (t <= bspline_interval_) {// First 3 control points in obstacles!
                            ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
                            return false;
                        }

                        break;
                    }
                }

                /*** collision check, phase 3 ***/
                if (!flag_occ) {
                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m",
                           iter_num_, time_ms, total_time_ms, final_cost);
                    success = true;
                } else {// restart
                    restart_nums++;
                    initControlPoints(cps_.points, false);
                    new_lambda2_ *= 2;
                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
                }
            } else if (result == lbfgs::LBFGSERR_CANCELED) {
                flag_force_return = true;
                rebound_times++;
                cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
            } else {
                ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
            }
        } while ((flag_occ || ellip_flag && restart_nums < MAX_RESART_NUMS_SET) ||
                 (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

        return success;
    }

    bool BsplineOptimizer::refine_optimize() {
        iter_num_ = 0;
        int start_id = order_;
        int end_id = cps_.points.cols() - order_;
        variable_num_ = 3 * (end_id - start_id);

        double q[variable_num_];
        double final_cost;

        memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

        double origin_lambda4 = lambda4_;
        bool flag_safe = true;
        int iter_count = 0;
        do {
            lbfgs::lbfgs_parameter_t lbfgs_params{};
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.001;

            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine,
                                               nullptr, nullptr, this, &lbfgs_params);
            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP) {
                //pass
            } else {
                ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
            }

            UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
            double tm, tmp;
            traj.getTimeSpan(tm, tmp);
            double length = (traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm();
            // Step size is defined as the maximum size that can passes through every gird.
            double t_step = (tmp - tm) / (length / grid_map_->getResolution());
            for (double t = tm; t < tmp * 2 / 3; t += t_step) {
                if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t))) {
                    Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
                    for (Eigen::Index i = 0; i < ref_pts_.size(); i++) {
                        ref_pts.row(i) = ref_pts_[i].transpose();
                    }

                    flag_safe = false;
                    break;
                }
            }

            if (!flag_safe)
                lambda4_ *= 2;

            iter_count++;
        } while (!flag_safe && iter_count <= 0);

        lambda4_ = origin_lambda4;

        //cout << "iter_num_=" << iter_num_ << endl;

        return flag_safe;
    }

    void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n) {
        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        /* ---------- evaluate cost and gradient ---------- */
        double f_smoothness, f_distance, f_feasibility /*, f_mov_objs*/, f_swarm, f_terminal;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
        // Eigen::MatrixXd g_mov_objs = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
        calcSwarmCost(cps_.points, f_swarm, g_swarm);
        calcTerminalCost(cps_.points, f_terminal, g_terminal);

        f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility +
                    new_lambda2_ * f_swarm + lambda2_ * f_terminal;
        f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
        // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility +
                                  new_lambda2_ * g_swarm + lambda2_ * g_terminal;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

    void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n) {

        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        /* ---------- evaluate cost and gradient ---------- */
        double f_smoothness, f_fitness, f_feasibility;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcFitnessCost(cps_.points, f_fitness, g_fitness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

        /* ---------- convert to solver format...---------- */
        f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;

        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

} // namespace ego_planner