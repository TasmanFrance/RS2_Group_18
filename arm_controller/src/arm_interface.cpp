#include "arm_interface.h"


///////////////////////////////////////////////////////////////////////////////
//Constructor
///////////////////////////////////////////////////////////////////////////////


ArmInterface::ArmInterface() :
Node("arm_Interface"), markerTolerance_(0.001)
{
    markerCount_ = 0;
    threadSleepSec_.store(100);

/*---------------------------------------------------------------------------*/
//Transform buffer and listener
/*---------------------------------------------------------------------------*/


    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


/*---------------------------------------------------------------------------*/
//Publishers
/*---------------------------------------------------------------------------*/


    //Markers
    //Called to output visuals for cones and road centre
    pub_vis_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker", 1000);


    pub_targetPoses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/target_poses", 10);


/*---------------------------------------------------------------------------*/
//Subscribers
/*---------------------------------------------------------------------------*/


    //Subscribing to canvas pose
    sub_centrePage_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/centerpage_marker", 10, std::bind(&ArmInterface::canvas_callback, this, std::placeholders::_1));


/*---------------------------------------------------------------------------*/


    //Subscribing to next draw point
    sub_drawPoses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/transformed_path", 10, std::bind(&ArmInterface::draw_callback, this, std::placeholders::_1));


/*---------------------------------------------------------------------------*/
//Thread
/*---------------------------------------------------------------------------*/


    //Creating new thread
    thread_ = new std::thread(&ArmInterface::threadProcess, this);
}


///////////////////////////////////////////////////////////////////////////////
//Destructor
///////////////////////////////////////////////////////////////////////////////


ArmInterface::~ArmInterface()
{
    if(thread_->joinable())
    {
        thread_->join();
    }
}


///////////////////////////////////////////////////////////////////////////////
//Callback functions
///////////////////////////////////////////////////////////////////////////////


void ArmInterface::canvas_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
{
    if (!msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Received null PoseStamped");
    }
    else
    {
        canvasCentreUpdated_ = true;
        std::unique_lock<std::mutex> lock (canvasPoseMtx_);
        canvasPose_ = *msg;
        {
            std::unique_lock<std::mutex> lock (canvasDistMtx_);
            canvasDist_ = canvasPose_.pose.position.z;
        }
    }
}


/*---------------------------------------------------------------------------*/


void ArmInterface::draw_callback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)
{
    if (!msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Received null PoseArray");
    }
    else
    {
        drawPathUpdated_ = true;
        std::unique_lock<std::mutex> lock (drawPosesMtx_);
        drawPoses_ = *msg;
    }
}


///////////////////////////////////////////////////////////////////////////////
//Main thread
///////////////////////////////////////////////////////////////////////////////


void ArmInterface::threadProcess()
{
    bool markersUpdated;
    while(rclcpp::ok())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "main thread");
        //Record whether any markers have been updated
        markersUpdated = (drawPathUpdated_ || canvasCentreUpdated_);
        markers_.markers.clear();

        if(drawPathUpdated_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "draw path update recieved, adding marker(s)");
            drawPathUpdated_ = false;

            
            try
            {
                geometry_msgs::msg::PoseArray tfDrawPoses = poseArrayTransform("tool0", "world", drawPoses_, 0.01, 0.05);
                geometry_msgs::msg::PoseArray upArray, downArray;

                bool upFlag = false;
                bool downFlag = false;
                for(unsigned int i=0 ; i < drawPoses_.poses.size() ; i++)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "adding marker at pose x: " << drawPoses_.poses.at(i).position.x << " y: " << drawPoses_.poses.at(i).position.y << " z: " << drawPoses_.poses.at(i).position.z);
                    if(abs(drawPoses_.poses.at(i).position.z - 1.0) < 1e-6)
                    {
                        upFlag = true;
                        upArray.poses.push_back(tfDrawPoses.poses.at(i));
                    }
                    else
                    {
                        downFlag = true;
                        downArray.poses.push_back(tfDrawPoses.poses.at(i));
                    }
                }
                //catching final line
                if(downFlag)
                {
                    addMarker(downArray, markerType::TPP_PEN_DOWN);
                    RCLCPP_INFO_STREAM(this->get_logger(), "adding pen down marker");
                    downArray.poses.clear();
                    downFlag = false;
                }
                if(upFlag)
                {
                    addMarker(upArray, markerType::TPP_PEN_UP);
                    RCLCPP_INFO_STREAM(this->get_logger(), "adding pen up marker");
                    upArray.poses.clear();
                    upFlag = false;
                }
            }
            catch(const std::exception& except)
            {
                RCLCPP_ERROR(this->get_logger(), "Canvas frame not ready: %s", except.what());
            }




//             try
//             {
//                 geometry_msgs::msg::PoseArray tfDrawPoses = poseArrayTransform("tool0", "world", drawPoses_, 0.01, 0.05);
//                 geometry_msgs::msg::PoseArray upArray, downArray;

//                 bool upFlag = false;
//                 bool downFlag = false;
//                 for(unsigned int i=0 ; i < drawPoses_.poses.size() ; i++)
//                 {
//                     RCLCPP_INFO_STREAM(this->get_logger(), "adding marker at pose x: " << drawPoses_.poses.at(i).position.x << " y: " << drawPoses_.poses.at(i).position.y << " z: " << drawPoses_.poses.at(i).position.z);
//                     if(abs(drawPoses_.poses.at(i).position.z - 1.0) < 1e-6)
//                     {
//                         upFlag = true;
//                         if(downFlag)
//                         {
//                             addMarker(downArray, markerType::TPP_PEN_DOWN);
//                             RCLCPP_INFO_STREAM(this->get_logger(), "adding pen down marker");
//                             downArray.poses.clear();
//                             downFlag = false;
//                         }
//                         upArray.poses.push_back(tfDrawPoses.poses.at(i));
//                     }
//                     else
//                     {
//                         downFlag = true;
//                         if(upFlag)
//                         {
//                             addMarker(upArray, markerType::TPP_PEN_UP);
//                             RCLCPP_INFO_STREAM(this->get_logger(), "adding pen up marker");
//                             upArray.poses.clear();
//                             upFlag = false;
//                         }
//                         downArray.poses.push_back(tfDrawPoses.poses.at(i));
//                     }
//                 }
//                 //catching final line
//                 if(downFlag)
//                 {
//                     addMarker(downArray, markerType::TPP_PEN_DOWN);
//                     RCLCPP_INFO_STREAM(this->get_logger(), "adding pen down marker");
//                     downArray.poses.clear();
//                     downFlag = false;
//                 }
//                 if(upFlag)
//                 {
//                     addMarker(upArray, markerType::TPP_PEN_UP);
//                     RCLCPP_INFO_STREAM(this->get_logger(), "adding pen up marker");
//                     upArray.poses.clear();
//                     upFlag = false;
//                 }
//             }
//             catch(const std::exception& except)
//             {
//                 RCLCPP_ERROR(this->get_logger(), "Canvas frame not ready: %s", except.what());
//             }

            // std::for_each(drawPoses_.poses.begin(),drawPoses_.poses.end(),[&](auto pose){
            //     RCLCPP_INFO_STREAM(this->get_logger(), "adding marker at pose x: " << pose.position.x << " y: " << pose.position.y << " z: " << pose.position.z);
            //     //Convert from local to global
            //     // pose.position.x += laserGlobalOffset.transform.translation.x;
            //     // pose.position.y += laserGlobalOffset.transform.translation.y;

            //     //Check if flagged as pen on canvas
            //     // std::cout << "Check cone flag: " << pose.position.z << std::endl;
            //     if(abs(pose.position.z - 1.0) < 1e-6)
            //     {
            //         ///???????????????????????????????????????????????????????????????????????????????CHANGE SO THAT IT IS z = 0 ACCOUNTING FOR ORIENTATION
            //         // // std::cout << "Is cone!" << std::endl;
            //         // unsigned int repeats = std::count_if(
            //         //     cones_.begin(),cones_.end(),[&](auto cone){
            //         //     return hypot(abs(cone.x-pose.position.x),abs(cone.y-pose.position.y)) < markerTolerance_;
            //         // });
            //         // if(repeats == 0)
            //         // {
            //             // std::cout << "Does not repeat" << std::endl;
            //             //New cone identified, adding to vector
            //             // pose.position.z = 0.0;
            //             // cones_.push_back(pose);
            //             RCLCPP_INFO_STREAM(this->get_logger(), "adding pen up marker");
            //             addMarker(pose, markerType::TPP_PEN_UP);
            //         // }
            //     }
            //     else
            //     {
            //         RCLCPP_INFO_STREAM(this->get_logger(), "adding pen down marker");
            //         addMarker(pose, markerType::TPP_PEN_UP);
            //     }
            // });
        }

        if(canvasCentreUpdated_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "canvas update recieved, adding marker");
            canvasCentreUpdated_ = false;
            geometry_msgs::msg::PoseStamped tfCanvasPose = poseStampedTransform("tool0", "world", canvasPose_);
            geometry_msgs::msg::PoseArray tfCanvasPoseConv;
            tfCanvasPoseConv.poses.push_back(tfCanvasPose.pose);
            addMarker(tfCanvasPoseConv, markerType::LOCAL_CANVAS_CENTRE);
        }

        if(markersUpdated)
        {
            try
            {
                pub_vis_marker_->publish(markers_);
                RCLCPP_INFO_STREAM(this->get_logger(), "publishing marker(s)");
            }
            catch(const std::exception& except)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to publish markers: %s", except.what());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(threadSleepSec_.load()));
    }
}


///////////////////////////////////////////////////////////////////////////////

        // try
        // {
        //     tf_buffer_->canTransform("world", "orange/laser_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
        // }
        // catch(const std::exception& except)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to get transform local laser and world: %s", except.what());
        //     gotWorldTf = false;
        //     return;
        // }

        // if(gotWorldTf)
        // {
        //     try
        //     {
        //         laserGlobalOffset = tf_buffer_->lookupTransform(
        //             "world","orange/laser_link", rclcpp::Time(0));
        //     }
        //     catch(tf2::TransformException& except)
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to transform laser offset point: %s", except.what());
        //         gotWorldTf = false;
        //         return;
        //     }
        // }




        // std::unique_lock<std::mutex> lock1 (canvasPoseMtx_);
        // std::unique_lock<std::mutex> lock2 (nextwaypointMtx_);

        // if(armControllerPtr_ == nullptr)
        // {
        //     armControllerPtr_ = std::make_unique<arm_controller>(canvasPose_);
        //     armControllerPtr_->init();
        // }
        // else
        // {
        //     armControllerPtr_->setNextPose(canvasPose_);
        // }

        // if(!armControllerPtr_->moveToNextPose())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        // }


///////////////////////////////////////////////////////////////////////////////


visualization_msgs::msg::MarkerArray ArmInterface::addMarker( const geometry_msgs::msg::PoseArray &poses, markerType type)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.lifetime = rclcpp::Duration(threadSleepSec_.load() / 1000, 0); // setting duration equal to detect objects thread refresh
    marker.id = markerCount_++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    // marker.pose.orientation = geometry_msgs::msg::Quaternion{0.0,0.0,0.0,1.0};
    // marker.scale = geometry_msgs::msg::Vector3{0.5,0.5,0.5};
    // marker.color = std_msgs::msg::ColorRGBA{0.5,0.0,0.0,0.0};

    switch(type)
    {
        case markerType::TPP_PEN_UP:
        {
            marker.ns = "pen_up";

            marker.color.r = 1.0;
            marker.color.g = 0.647;

            if(poses.poses.size() > 1)
            {
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

                for (const auto& pose : poses.poses)
                {
                    geometry_msgs::msg::Point point;
                    point.x = pose.position.x;
                    point.y = pose.position.y;
                    point.z = pose.position.z;

                    marker.points.push_back(point);
                }
            }
            else
            {
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.pose = poses.poses.at(0);
            }
            break;
        }
        case markerType::TPP_PEN_DOWN:
        {
            marker.ns = "pen_down";

            marker.color.r = 1.0;

            if(poses.poses.size() > 1)
            {
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

                for (const auto& pose : poses.poses)
                {
                    geometry_msgs::msg::Point point;
                    point.x = pose.position.x;
                    point.y = pose.position.y;
                    point.z = pose.position.z;

                    marker.points.push_back(point);
                }
            }
            else
            {
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.pose = poses.poses.at(0);
            }
            break;
        }
        case markerType::LOCAL_CANVAS_CENTRE:
        {
            marker.ns = "canvas_centre";
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.color.a = 0.1;
            marker.color.g = 1.0;

            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.001;

            marker.pose = poses.poses.at(0);
            break;
        }
    }

    // std::cout << "Adding id " << marker.id << " type " << marker.ns << " at x: " << marker.pose.position.x << " y: " << marker.pose.position.y << " z: " << marker.pose.position.z << std::endl;

    // unsigned int repeats = std::count_if(
    //     markers_.begin(),markers_.end(),[&](auto cone){
    //         return abs(cone.x-pose.x)<1e-6 &&
    //                 abs(cone.y-pose.y)<1e-6;
    // });
    //     if(repeats == 0)
    //     {
    //         std::cout << "Does not repeat" << std::endl;
    //         //New cone identified, adding to vector
    //         pose.z = 0.0;
    //         cones_.push_back(pose);
    //         addMarker(pose, markerType::OBJECT_CONE);
    //     }
    // }

    RCLCPP_INFO(this->get_logger(), "Publishing marker with ID: %d, Namespace: %s", marker.id, marker.ns.c_str());

    markers_.markers.push_back(marker);

    return markers_;
}


///////////////////////////////////////////////////////////////////////////////


geometry_msgs::msg::PoseArray ArmInterface::poseArrayTransform(const std::string &currentFrameID, const std::string &desiredFrameID, const geometry_msgs::msg::PoseArray &poses, const double &scaling, const double &zHeight)
{
    geometry_msgs::msg::PoseArray transfPoses;
    transfPoses.header.stamp = this->now();
    transfPoses.header.frame_id = desiredFrameID;

    try
    {
        // Lookup the transform from the end effector frame to the world frame
        geometry_msgs::msg::TransformStamped tfStamped = tf_buffer_->lookupTransform(desiredFrameID, currentFrameID, tf2::TimePointZero);

        // Transform each pose in the pose array
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        std::for_each(poses.poses.begin(), poses.poses.end(), [&](auto pose){
            pose_in.header.stamp = this->now();   // Set the current time
            pose_in.header.frame_id = currentFrameID;
            pose_in.pose.position.x = pose.position.x * scaling;
            pose_in.pose.position.y = pose.position.y * scaling;

            {
                //Change pen z based on distance from page
                std::unique_lock<std::mutex> lock (canvasDistMtx_);
                pose_in.pose.position.z = canvasDist_ - pose.position.z * zHeight;
            }

            tf2::doTransform(pose_in, pose_out, tfStamped);
            transfPoses.poses.push_back(pose_out.pose);
        });
    }
    catch (tf2::TransformException &except)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose array: %s", except.what());
    }

    return transfPoses;
}


///////////////////////////////////////////////////////////////////////////////


geometry_msgs::msg::PoseStamped ArmInterface::poseStampedTransform(const std::string &currentFrameID, const std::string &desiredFrameID, const geometry_msgs::msg::PoseStamped &pose)
{
    geometry_msgs::msg::PoseStamped transfPose;
    transfPose.header.stamp = this->now();
    transfPose.header.frame_id = desiredFrameID;

    try
    {
        // Lookup the transform from the current frame to the desired frame
        geometry_msgs::msg::TransformStamped tfStamped = tf_buffer_->lookupTransform(desiredFrameID, currentFrameID, tf2::TimePointZero);

        // Transform the pose
        tf2::doTransform(pose, transfPose, tfStamped);
    }
    catch (tf2::TransformException &except)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", except.what());
    }

    return transfPose;
}