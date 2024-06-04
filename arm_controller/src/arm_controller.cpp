#include "arm_controller.h"


///////////////////////////////////////////////////////////////////////////////
//Constructor
///////////////////////////////////////////////////////////////////////////////


ArmController::ArmController() :
Node("arm_controller", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
runPath_(false)
// thread_(new std::thread(&ArmController::threadRun, this)),
// move_group_interface_(this, PLANNING_GROUP_ARM_)
{
/*---------------------------------------------------------------------------*/
//Init variables
/*---------------------------------------------------------------------------*/

    //Create the MoveIt MoveGroup Interface
    // move_group_interface_ = moveit::planning_interface::MoveGroupInterface(this, PLANNING_GROUP_ARM_);
    // move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    //     this, PLANNING_GROUP_ARM_, nullptr);
        // , rclcpp::Duration(0));

/*---------------------------------------------------------------------------*/
//Publishers
/*---------------------------------------------------------------------------*/


    pub_armReady_ = this->create_publisher<std_msgs::msg::Bool>(
        "/arm_ready", 1);


/*---------------------------------------------------------------------------*/
//Subscribers
/*---------------------------------------------------------------------------*/


    sub_targetPoses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/target_poses", 10, std::bind(&ArmController::target_callback, this, std::placeholders::_1));


/*---------------------------------------------------------------------------*/
//Thread
/*---------------------------------------------------------------------------*/


    //Creating new thread
    thread_ = new std::thread(&ArmController::threadRun, this);
}


///////////////////////////////////////////////////////////////////////////////
//Destructor
///////////////////////////////////////////////////////////////////////////////


ArmController::~ArmController()
{
    if(thread_->joinable())
    {
        thread_->join();
        rclcpp::shutdown();
    }
}


///////////////////////////////////////////////////////////////////////////////
//Callback functions
///////////////////////////////////////////////////////////////////////////////


void ArmController::target_callback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)
{
    if (!msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Received null PoseArray");
    }
    else
    {
        runPath_ = true;
        std::unique_lock<std::mutex> lock(targetPosesMtx_);
        targetPoses_ = *msg;
        pub_armReady_->publish([]() {
                std_msgs::msg::Bool msg;
                msg.data = false;
                return msg;
            }());
    }
}


///////////////////////////////////////////////////////////////////////////////
//Main thread
///////////////////////////////////////////////////////////////////////////////


void ArmController::threadRun()
{
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = std::make_shared<rclcpp::Node>("arm_moveit", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group_arm = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP_ARM_);
    while(rclcpp::ok())
    {
        // if(firstRun_)
        // {
        //     rclcpp::NodeOptions node_options;
        //     node_options.automatically_declare_parameters_from_overrides(true);

        //     auto move_group_node_ = std::make_shared<rclcpp::Node>("arm_moveit", node_options);

        //     rclcpp::executors::SingleThreadedExecutor executor;
        //     executor.add_node(move_group_node_);
        //     std::thread([&executor]() { executor.spin(); }).detach();

        //     firstRun_ = false;
        //     // auto move_group_interface = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP_ARM_);
        // }
        if(runPath_)
        {
            if(abs(targetPoses_.poses.at(0).position.z - 1.0) < 1e-6)
            {
                RCLCPP_INFO(this->get_logger(), "Move off page");
                //If the pen is above the page move using ikine
                //Set a target Pose
                //(this is where the end effector translations would be applied for D435i and pen position)
                move_group_arm.setPoseTarget(targetPoses_.poses.at(0));


                //Create a plan to that target pose
                // auto const move_group_arm = move_group_arm;
                // auto const [success, plan] = [&move_group_arm]
                auto const [success, plan] = [&]
                {
                    moveit::planning_interface::MoveGroupInterface::Plan msg;
                    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
                    return std::make_pair(ok, msg);
                }();


                //Execute the plan
                if(success)
                {
                    move_group_arm.execute(plan);
                    targetPoses_.poses.erase((targetPoses_.poses.begin()));
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Draw on page");
                //Pen down, using RMRC
                std::vector<geometry_msgs::msg::Pose> waypoints;
                std::vector<geometry_msgs::msg::Pose>::iterator lastIter = targetPoses_.poses.begin();

                for (auto it = targetPoses_.poses.begin(); it != targetPoses_.poses.end(); ++it) {
                    if(!(it->position.z - 1.0) < 1e-6)
                    {
                        waypoints.push_back(*it);
                        lastIter = it;
                    }
                    else
                    {
                        //Only include until pen is pulled up or vector finishes
                        it = targetPoses_.poses.end();
                    }
                }

                double eef_step = 0.01;
                double jump_threshold = 0.0;
                moveit_msgs::msg::RobotTrajectory trajectory;
                bool avoid_collision = true;
                moveit_msgs::msg::MoveItErrorCodes* error_code = nullptr;

                // Computing the Cartesian path, which is stored in trajectory
                double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collision, error_code);
                RCLCPP_INFO(this->get_logger(), "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

                // Check if complete path is possible and execute the trajectory
                if(fraction == 1)
                {
                    //Execute
                    move_group_arm.execute(trajectory);
                    for (auto it = targetPoses_.poses.begin(); it != lastIter; ++it)
                    {
                        //Removing all poses that have been executed
                        targetPoses_.poses.erase(it);
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Cartesian path plan failed with error code: %d", error_code->val);
                }

            }

            if(targetPoses_.poses.size() == 0)
            {
                runPath_ = false;
                pub_armReady_->publish([]() {
                    std_msgs::msg::Bool msg;
                    msg.data = true;
                    return msg;
                }());
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    }
}


///////////////////////////////////////////////////////////////////////////////
//Functions
///////////////////////////////////////////////////////////////////////////////


// bool ArmController::moveToNextPose()
// {
    // //Set a target Pose
    // //(this is where the end effector translations would be applied for D435i and pen position)
    // move_group_interface_.setPoseTarget(targetPose_);


    // //Create a plan to that target pose
    // // auto const move_group_interface = move_group_interface_;
    // // auto const [success, plan] = [&move_group_interface_]
    // auto const [success, plan] = [&]
    // {
    //     moveit::planning_interface::MoveGroupInterface::Plan msg;
    //     auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
    //     return std::make_pair(ok, msg);
    // }();


    // //Execute the plan
    // if(success)
    // {
    //     move_group_interface_.execute(plan);
    // }


//     return success;
// }


///////////////////////////////////////////////////////////////////////////////