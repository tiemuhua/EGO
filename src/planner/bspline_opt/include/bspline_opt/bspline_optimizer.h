#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner {

    class ControlPoints {
    public:
        double clearance;
        int size;
        Eigen::MatrixXd points;
        std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
        std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
        std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

        void resize(const int size_set) {
            size = size_set;

            base_point.clear();
            direction.clear();
            flag_temp.clear();

            points.resize(3, size_set);
            base_point.resize(size);
            direction.resize(size);
            flag_temp.resize(size);
        }

        void segment(ControlPoints &buf, const int start, const int end) {

            if (start < 0 || end >= size || points.rows() != 3) {
                ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
                return;
            }

            buf.resize(end - start + 1);
            buf.points = points.block(0, start, 3, end - start + 1);
            buf.clearance = clearance;
            buf.size = end - start + 1;
            for (int i = start; i <= end; i++) {
                buf.base_point[i - start] = base_point[i];
                buf.direction[i - start] = direction[i];
            }
        }
    };

    class BsplineOptimizer {

    public:
        BsplineOptimizer() = default;

        ~BsplineOptimizer() = default;

        /* main API */
        void setEnvironment(const GridMap::Ptr &map);

        void setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr &mov_obj);

        void setParam(ros::NodeHandle &nh);

        /* helper function */

        // required inputs
        void setControlPoints(const Eigen::MatrixXd &points);

        inline void setBsplineInterval(const double &ts);

        void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);

        void setDroneId(int drone_id);

        // optional inputs
        void setLocalTargetPt(const Eigen::Vector3d &local_target_pt) { local_target_pt_ = local_target_pt; };

        AStar::Ptr a_star_;

        std::vector<ControlPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

        std::vector<std::pair<int, int>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);

        // must be called after initControlPoints()
        bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts);

        bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost,
                                        const ControlPoints &control_points, double ts);

        bool BsplineOptimizeTrajRefine(vector<Eigen::Vector3d> &&ref_pts, const Eigen::MatrixXd &init_cps,
                                       double ts, Eigen::MatrixXd &optimal_cps);

        inline double getSwarmClearance() const { return swarm_clearance_; }

    private:
        GridMap::Ptr grid_map_;
        fast_planner::ObjPredictor::Ptr moving_objs_;
        SwarmTrajData *swarm_trajs_{nullptr}; // Can not use shared_ptr and no need to free
        int drone_id_{};

        enum FORCE_STOP_OPTIMIZE_TYPE {
            DONT_STOP,
            STOP_FOR_REBOUND,
            STOP_FOR_ERROR
        } force_stop_type_;

        // main input
        double bspline_interval_{};           // B-spline knot span

        /* optimization parameters */
        int order_{};                           // bspline degree
        double lambda1_{};                      // jerk smoothness weight
        double lambda2_{}, new_lambda2_{};      // distance weight
        double lambda3_{};                      // feasibility weight
        double lambda4_{};                      // curve fitting

        double dist0_{}, swarm_clearance_{};    // safe distance
        double max_vel_{}, max_acc_{};          // dynamic limits

        int variable_num_{};                    // optimization variables
        int iter_num_{};                        // iteration of the solver

        Eigen::Vector3d local_target_pt_;

#define INIT_min_ellip_dist_ 123456789.0123456789
        double min_ellip_dist_{};

        ControlPoints cps_;
        std::vector<Eigen::Vector3d> ref_pts_;

        /* cost function */
        /* calculate each part of cost function with control points q as input */
        // q contains all control points
        static void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient,
                                       bool falg_use_jerk = true);

        void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient) const;

        void calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

        void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num,
                                     double smoothness_cost);

        void calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

        void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient) const;

        bool check_collision_and_rebound();

        static int earlyExit(void *func_data, const double *x, const double *g, double fx, double xnorm, double gnorm,
                             double step, int n, int k, int ls);

        static double costFunctionRebound(void *func_data, const double *x, double *grad, int n);

        static double costFunctionRefine(void *func_data, const double *x, double *grad, int n);

        bool rebound_optimize(double &final_cost);

        bool refine_optimize();

        void combineCostRebound(const double *x, double *grad, double &f_combine, int n);

        void combineCostRefine(const double *x, double *grad, double &f_combine, int n);

        bool reverseCpsBasePoint(const double RESOLUTION, const double CTRL_PT_DIST,
                                 const ControlPoints &origin_cps, ControlPoints &reversed_cps, bool &error);

        bool reverseBasePointForSingleControlPoint(const double RESOLUTION, const double CTRL_PT_DIST,
                                                   const ControlPoints &origin_cps, ControlPoints &reversed_cps);

        void getBasePointAndDirectionForSegment(const int start_id, const int end_id,
                                                ControlPoints &cps, bool &a_star_success, bool &base_point_success);
        /* for benckmark evaluation only */
    public:
        typedef unique_ptr<BsplineOptimizer> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace ego_planner
#endif