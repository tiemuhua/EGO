#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace ego_planner {

    // Fast Planner Manager
    // Key algorithms of mapping and planning are called

    class EGOPlannerManager {
        // SECTION stable
    public:
        EGOPlannerManager();

        ~EGOPlannerManager();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* main planning interface */
        bool reboundReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                           const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, bool flag_polyInit, bool flag_randomPolyTraj);

        bool EmergencyStop(const Eigen::Vector3d &stop_pos);

        bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                            const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

        void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

        void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

        void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

        double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

        bool checkCollision(int drone_id);


        PlanParameters pp_;
        LocalTrajData local_data_;
        GlobalTrajData global_data_;
        GridMap::Ptr grid_map_;
        fast_planner::ObjPredictor::Ptr obj_predictor_;
        SwarmTrajData swarm_trajs_buf_;

    private:
        /* main planning algorithms & modules */
        PlanningVisualization::Ptr visualization_;

        BsplineOptimizer::Ptr bspline_optimizer_;

        int continuous_failures_count_{0};

        bool initPathFromPreviousTrajectory(const double ts, bool &init_fail,
                                            const Eigen::Vector3d& local_target_pt, const Eigen::Vector3d &local_target_vel,
                                            vector<Eigen::Vector3d> &start_end_derivatives, vector<Eigen::Vector3d> &point_set);

        bool initPathAsPoly(double &ts, const bool flag_randomPolyTraj,
                            const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                            const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
                            vector<Eigen::Vector3d> &start_end_derivatives, vector<Eigen::Vector3d> &point_set);


        void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

        static void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                   Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc);

        bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts,
                            Eigen::MatrixXd &optimal_control_points);

    public:
        typedef unique_ptr<EGOPlannerManager> Ptr;
    };
} // namespace ego_planner

#endif