#include "vfh_local_planner/vfh_local_planner_ros.h"

#include <pluginlib/class_list_macros.h>

// PLUGINLIB_DECLARE_CLASS(vfh_local_planner, VFHPlannerRos, vfh_local_planner::VFHPlannerRos, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(vfh_local_planner::VFHPlannerRos, nav_core::BaseLocalPlanner)

namespace tf2 {
    void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2)
{
    tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

    void transformMsgToTF2(const geometry_msgs::PoseStamped& msg, tf2::Transform& tf2) {
        tf2 = tf2::Transform(tf2::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w), tf2::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
    }

    void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg)
    {
    msg.translation.x = tf2.getOrigin().x();
    msg.translation.y = tf2.getOrigin().y();
    msg.translation.z = tf2.getOrigin().z();
    msg.rotation.x = tf2.getRotation().x();
    msg.rotation.y = tf2.getRotation().y();
    msg.rotation.z = tf2.getRotation().z();
    msg.rotation.w = tf2.getRotation().w();
    }
}
namespace vfh_local_planner
{
    VFHPlannerRos::VFHPlannerRos(): costmap_ros_(NULL), tf_(), initialized_(false), odom_helper_("odom") {};

    VFHPlannerRos::VFHPlannerRos(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_ros_(NULL), tf_(), initialized_(false), odom_helper_("odom")
    {
        // initialize planner
		initialize(name, tf, costmap_ros);
    };

    VFHPlannerRos::~VFHPlannerRos() {};

    void VFHPlannerRos::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    // void VFHPlannerRos::initialize(std::string name, tf2_ros::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // check if plugin initialized
        if(!initialized_)
		{
            ROS_INFO("Initializing VFH Planner");

            ros::NodeHandle private_nh("~/" + name);

            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

            //Parameter for dynamic reconfigure
            dsrv_ = new dynamic_reconfigure::Server<vfh_local_plannerConfig>(private_nh);
            dynamic_reconfigure::Server<vfh_local_plannerConfig>::CallbackType cb = boost::bind(&VFHPlannerRos::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            xy_goal_latch_ = false;
            rotating_to_goal_ = false;
            finding_alternative_way_ = false;

            costmap_ros_ = costmap_ros;
            // tf_ = new tf2_ros::TransformListener(buffer_);
            tf_ = tf;

            global_frame_ = costmap_ros_->getGlobalFrameID();
            std::cout << "costmapa_frame_id: " << costmap_ros->getBaseFrameID() << " " << costmap_ros->getGlobalFrameID() << std::endl;
            costmap_ = costmap_ros_->getCostmap();
            initialized_ = true;

            vfh_planner.Initialize(costmap_);     
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
        
    }

    void VFHPlannerRos::reconfigureCB(vfh_local_plannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
        vfh_planner.Reconfigure(config);
        goal_reached_ = false;
    }

    bool VFHPlannerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

        ROS_INFO("Got new plan!");
        //reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        xy_goal_latch_ = false;
        rotating_to_goal_ = true;
        goal_reached_ = false;
        finding_alternative_way_ = false;
            
        return true;
    };

    bool VFHPlannerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        ROS_INFO("computeVelocityCommands");
        //Check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

        std::vector<geometry_msgs::PoseStamped> local_plan;
        //Get the pose of the robot in the global frame of the costmap
        // tf2::Stamped<tf2::Pose> current_pose;
        geometry_msgs::PoseStamped current_pose;
        if (!costmap_ros_->getRobotPose(current_pose)) {
            std::cout << "cant get current pose" << std::endl;
            return false;
        }

        //Get current robot velocity
        // tf::Stamped<tf::Pose> current_vel;
        geometry_msgs::PoseStamped current_vel;
        odom_helper_.getRobotVel(current_vel);

        //Create transform used to transform coordinates from global planner frame to costmap frame
        std::cout << "global_frame_id: " << global_frame_ << " plan_frame_id: " << global_plan_.back().header.frame_id << std::endl;
        // ROS_INFO("global_frame_id: %s, plan_frame_id: %s", global_frame_., global_plan_.back().header.frame_id);
        ros::Time now = ros::Time::now();
 
        geometry_msgs::TransformStamped frame_transform = tf_->lookupTransform(global_frame_, now, global_plan_.back().header.frame_id, now, global_frame_, ros::Duration(1));
        
        //Transform plan goal from the global planner frame to the frame of the costmap (global_frame_ = costmap_frame)
        const geometry_msgs::PoseStamped& plan_goal = global_plan_.back();
        tf2::Stamped<tf2::Pose> global_goal;
        geometry_msgs::PoseStamped ps;

        ps = tf_->transform(plan_goal, global_frame_);
        tf2::Transform psT;
        tf2::transformMsgToTF2(ps, psT);
        
        global_goal.setData(psT);
        global_goal.stamp_ = frame_transform.header.stamp;
        global_goal.frame_id_ = global_frame_;
        
        //Transforms the global plan of the robot from the global planner frame to the frame of the costmap
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, current_pose, *costmap_, global_frame_, transformed_plan)) {           
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            if (!config_.ignore_global_plan_updates)
                return false;
        }

        tf2::Stamped<tf2::Pose> intermediary_goal_point;
        //Check if the modified plan is not empty
        if (!transformed_plan.empty())
        {
            //Trim off parts of the global plan that are far enough behind the robot
            //base_local_planner::prunePlan(current_pose, transformed_plan, global_plan_);

            //Get intermediary goal point in the transformed plan
            tf2::Stamped<tf2::Pose> current_pose_n;
            tf2::transformMsgToTF2(current_pose, current_pose_n);
            int point_index = GetPlanPoint(transformed_plan, global_plan_,current_pose_n);
            // tf2::poseStampedMsgToTF(transformed_plan.at(point_index), intermediary_goal_point);
            tf2::transformMsgToTF2(transformed_plan.at(point_index), intermediary_goal_point);
            std::cout << "intermediary_goal_point: (" << intermediary_goal_point.getOrigin().getX() << ", " << intermediary_goal_point.getOrigin().getY() << ")" << std::endl;
            std::cout << "current_pose_n: (" << current_pose_n.getOrigin().getX() << ", " << current_pose_n.getOrigin().getY() << ")" << std::endl;

        }
        else
        {
            ROS_ERROR("Plan is empty");
            if (!config_.ignore_global_plan_updates)
                return false;
        }
        
        //Update VFH histogram with new costmap
        if (!vfh_planner.UpdateHistogram())
        {
            ROS_WARN("Could not find clear direction");
            return false;
        }

        //#############################################################################################################################################
        //########################################### Check if the robot reached the goal position ####################################################

        //Check if the robot is at the goal coordinate x y
        double goal_x = global_goal.getOrigin().getX();
        double goal_y = global_goal.getOrigin().getY();
        tf2::Stamped<tf2::Pose> current_pose_n, current_vel_n;
        tf2::transformMsgToTF2(current_pose, current_pose_n);
        tf2::transformMsgToTF2(current_vel, current_vel_n);
        if (base_local_planner::getGoalPositionDistance(current_pose, goal_x, goal_y) <= config_.xy_goal_tolerance_ || xy_goal_latch_)
        {
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishPlan(transformed_plan, g_plan_pub_);
            publishPlan(local_plan, l_plan_pub_);

            std::cout << "xy reached" << std::endl;
            xy_goal_latch_ = true;
            //Check if the robot is at the same orientation of the goal
            double goal_th = tf2::impl::getYaw(intermediary_goal_point.getRotation());
            if (fabs(base_local_planner::getGoalOrientationAngleDifference(current_pose, goal_th)) <= config_.yaw_goal_tolerance_)
            {
                std::cout << "parou yupi" << std::endl;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                goal_reached_ = true;
                return true;
            }
            else{
                ROS_INFO("################## rotating to goal to end ############################");
                std::cout << "rotating to goal to end" << std::endl;
                
                vfh_planner.RotateToGoal(current_pose_n, current_vel_n, goal_th, cmd_vel);
                return true;
            }
        }
        //#############################################################################################################################################
        //######################################################## Drive to the goal ##################################################################
        ROS_INFO("########################## Drive to the goal #######################################");
        double direction_to_follow;
        double goal_distance;
        // if(finding_alternative_way_)
        // {
            
            // double global_plan_goal_angle = atan2((global_goal.getOrigin().getY()-current_pose_n.getOrigin().getY()), (global_goal.getOrigin().getX()-current_pose_n.getOrigin().getX()));
            double local_plan_goal_angle = atan2((intermediary_goal_point.getOrigin().getY()-current_pose_n.getOrigin().getY()), (intermediary_goal_point.getOrigin().getX()-current_pose_n.getOrigin().getX()));
            goal_distance = 0.2;
            if (vfh_planner.DirectionIsClear(local_plan_goal_angle, intermediary_goal_point, current_pose_n))
            {
                ROS_INFO("==========get goal direction=========");
                direction_to_follow = local_plan_goal_angle;
                goal_distance = std::min(sqrt(pow((intermediary_goal_point.getOrigin().getX()-current_pose_n.getOrigin().getX()),2)+pow((intermediary_goal_point.getOrigin().getY()-current_pose_n.getOrigin().getY()),2)), 0.3);
            }
            else
            {
                ROS_INFO("==========get new direction=========");
                // 获取规划方向
                direction_to_follow = vfh_planner.GetNewDirection(
                    local_plan_goal_angle, 
                    tf2::impl::getYaw(current_pose_.getRotation()),
                    previews_direction,
                    current_pose_
                    );
            }
        // }
        /*
        else
        {
            double intermediary_goal_orientation = atan2((intermediary_goal_point.getOrigin().getY()-current_pose_n.getOrigin().getY()), (intermediary_goal_point.getOrigin().getX()-current_pose_n.getOrigin().getX()));

            //Check if the path is free
            if (!vfh_planner.DirectionIsClear(intermediary_goal_orientation, intermediary_goal_point))
            {
                ROS_INFO("==========get goal direction2=========");
                finding_alternative_way_ = true;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }
            else
            {
                ROS_INFO("==========get new direction2=========");
                direction_to_follow = intermediary_goal_orientation;
                goal_distance = sqrt(pow((intermediary_goal_point.getOrigin().getX()-current_pose_n.getOrigin().getX()),2)+pow((intermediary_goal_point.getOrigin().getY()-current_pose_n.getOrigin().getY()),2));
            }
        }
        */
        previews_direction = direction_to_follow;

        std::cout << "Going to direction: " << radToDeg(direction_to_follow) << std::endl;
        if (rotating_to_goal_)
        {
            std::cout << "rotating to goal to start1" << std::endl;
            vfh_planner.RotateToGoal(current_pose_n, current_vel_n, direction_to_follow, cmd_vel);
            if (fabs(base_local_planner::getGoalOrientationAngleDifference(current_pose, direction_to_follow)) < config_.yaw_goal_tolerance_)
            {
                rotating_to_goal_ = false;
            }
            return true;
        }
        //Check if the robot is deviating too much from the plan
        else if (fabs(base_local_planner::getGoalOrientationAngleDifference(current_pose, direction_to_follow)) > M_PI/3)
        {
            std::cout << "rotating to goal to start2" << std::endl;
            rotating_to_goal_ = true;
            cmd_vel.linear.x = 0.05;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        //Drive toward the plan
        else 
        {
            std::cout << "driving to goal" << std::endl;
            vfh_planner.DriveToward(angles::shortest_angular_distance(tf2::impl::getYaw(current_pose_n.getRotation()),direction_to_follow), goal_distance, cmd_vel);
        }

        //publish information to the visualizer
        getLocalPlan(direction_to_follow, local_plan);
        publishPlan(transformed_plan, g_plan_pub_);
        publishPlan(local_plan, l_plan_pub_);

        return true;
    };

    bool VFHPlannerRos::isGoalReached()
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
        return goal_reached_;
    };

    // int VFHPlannerRos::GetPlanPoint(std::vector<geometry_msgs::PoseStamped> transformed_plan, std::vector<geometry_msgs::PoseStamped> &global_plan, tf::Stamped<tf::Pose> current_pose)
    int VFHPlannerRos::GetPlanPoint(
        std::vector<geometry_msgs::PoseStamped> transformed_plan, 
        std::vector<geometry_msgs::PoseStamped> &global_plan, 
        tf2::Stamped<tf2::Pose> current_pose
        )
    {
        return transformed_plan.size() / 2;
        // int point = 0;
        // for (int i = 0; i < transformed_plan.size(); i++)
        // {
        //     double point_distance = sqrt(pow((transformed_plan.at(i).pose.position.x-current_pose.getOrigin().getX()),2)+pow((transformed_plan.at(i).pose.position.y-current_pose.getOrigin().getY()),2));
        //     if (point_distance < 0.6)
        //     {
        //         point = i;
        //         return point;
        //     }           
        // }

    }

    void VFHPlannerRos::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {
        //given an empty path we won't do anything
        if(path.empty())
        return;

        //create a path message
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for(unsigned int i=0; i < path.size(); i++){
        gui_path.poses[i] = path[i];
        }

        pub.publish(gui_path);
    }

    void VFHPlannerRos::getLocalPlan( double direction_to_follow, std::vector<geometry_msgs::PoseStamped>& local_plan) {
        geometry_msgs::PoseStamped next_pose;
        float step = 0.05;
        for (int i = 0; i < 10; i++) {
            next_pose.header.frame_id = global_frame_;
            next_pose.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped current_pose;
            if (!costmap_ros_->getRobotPose(current_pose)) {
                std::cout << "cant get current pose" << std::endl;
                return;
            }
            // next_pose.pose.position.x = current_pose_.getOrigin().getX() + i * step * std::cos(direction_to_follow);
            // next_pose.pose.position.y = current_pose_.getOrigin().getY() + i * step * std::sin(direction_to_follow);
            next_pose.pose.position.x = current_pose.pose.position.x + i * step * std::cos(direction_to_follow);
            next_pose.pose.position.y = current_pose.pose.position.y + i * step * std::sin(direction_to_follow);

            tf2::Quaternion qtn;
            qtn.setRPY(0, 0, direction_to_follow);
            next_pose.pose.orientation.x = qtn.getX();
            next_pose.pose.orientation.y = qtn.getY();
            next_pose.pose.orientation.z = qtn.getZ();
            next_pose.pose.orientation.w = qtn.getW();

            local_plan.push_back(next_pose);
        }
    }
}
