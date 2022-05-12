#ifndef VFH_LOCAL_PLANNER_H_
#define VFH_LOCAL_PLANNER_H_

#include <stdio.h>
#include <vector>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>
#include <angles/angles.h>

#include <ros/ros.h>
#include "vfh_local_planner/utils.h"
#include "vfh_local_planner/vfh_local_plannerConfig.h"

// #include "transform_datatypes.h"
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2/impl/utils.h"


namespace tf2 {
    typedef tf2::Vector3 Point;
    typedef tf2::Transform Pose;
}
namespace vfh_local_planner
{
    class VFHPlanner
    {
    public:
        VFHPlanner();
        ~VFHPlanner();

        void Reconfigure(vfh_local_plannerConfig &cfg);
        bool Initialize(costmap_2d::Costmap2D* costmap);
        void Alocate();
        bool UpdateHistogram();
        void SmoothHistogram();
        void GetCandidateValleys();

        bool RotateToGoal(const tf2::Stamped<tf2::Pose>& global_pose, const tf2::Stamped<tf2::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
        void DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::Twist& cmd_vel);
        bool DirectionIsClear(double goal_direction, tf2::Stamped<tf2::Pose> goal_pose, tf2::Stamped<tf2::Pose> current_pose);
        // double GetNewDirection(double global_plan_goal_direction, double current_robot_direction, double previews_direction);
        double GetNewDirection(
            double global_plan_goal_direction, 
            double current_robot_direction, 
            double previews_direction, 
            tf2::Stamped<tf2::Pose> pose
            );

        void updatePassPose(tf2::Stamped<tf2::Pose> pose);

        /**
         * @brief judge whether the robot back and forth in the direction
         * @param deviation_angle the drive direction
         * @return True if shock
         */
        bool judgeShock(double goal_direction);
        
        /**
         * @brief judge whether the robot Collision with obstacles at next time
         * @return True if the robot will crash with obstacle
        */
        bool judgeIntersect();

        /**
        * @brief judge the direction is direct to goal
        * @param deviation_angle the goal direction
        * @return True if robot can drive directily to the goal
        * */
        bool isDirect(double deviation_angle, tf2::Stamped<tf2::Pose> goal_pose, tf2::Stamped<tf2::Pose> current_pose);

        bool isDirect(tf2::Stamped<tf2::Pose> goal_pose, tf2::Stamped<tf2::Pose> current_pose);

    private:
        int window_width;
        int window_height;

        int vfh_sections_number;
        double vhf_detection_range, increase_rate;
        int goal_weight, curr_direction_weight, prev_direction_weight;
        int smooth_length;
        double vfh_threshold;
        int wide_valley_threshold;
        int very_narrow_valley_threshold;
        double max_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_;
        double acc_lim_x_, acc_lim_theta_;
        double local_planner_frequence;

        double cmd_vel_linear_x_;
        double cmd_vel_angular_z_;

        std::vector<geometry_msgs::Point> pass_pos_;
        // geometry_msgs::PoseStamped current_pose_;

        costmap_2d::Costmap2D* costmap_;
        std::vector<std::vector<double> > costmap_cells_angle;
        std::vector<std::vector<double> > costmap_cells_distance;
        std::vector<double> vfh_histogram;
        std::vector<std::vector<int> > candidate_valleys;
        std::vector<int> rejected_peaks;
        vfh_local_planner::vfh_local_plannerConfig config_;

    };
}

#endif
