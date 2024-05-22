#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

// #include <moveit/planning_interface/planning_in terface.h>

class ArucoLocalisationNode : public rclcpp::Node
{
public:
	ArucoLocalisationNode()
		: Node("aruco_localisation")
	{
		depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/camera/camera/depth/image_rect_raw",
			10,
			[this](const sensor_msgs::msg::Image::SharedPtr msg)
			{
				// Process depth map
				processDepthMap(msg);
			});

		color_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/camera/camera/color/image_raw",
			10,
			[this](const sensor_msgs::msg::Image::SharedPtr msg)
			{
				// Process color image
				processColorImage(msg);
			});
		camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			"/camera/camera/aligned_depth_to_color/camera_info",
			10,
			[this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
			{
				// Update camera matrix and distortion coefficients
				updateCameraCalibration(msg);
			});
		marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
			"detected_markers",
			10);
		center_marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
			"centerpage_marker",
			10);
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_marker_publisher_;

	cv::Mat camera_matrix_;
	cv::Mat distortion_coeffs_;

	void processDepthMap(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		// Check if the image is in the expected format
		if (msg->encoding != "16UC1")
		{
			RCLCPP_WARN(this->get_logger(), "Unexpected image encoding: %s", msg->encoding.c_str());
			return;
		}

		// Convert sensor_msgs::Image to cv::Mat
		cv::Mat depth_image(msg->height, msg->width, CV_16UC1, const_cast<unsigned char *>(msg->data.data()), msg->step);

		// Apply Gaussian blur to the depth image
		cv::Mat blurred_depth_image;
		cv::GaussianBlur(depth_image, blurred_depth_image, cv::Size(5, 5), 0); // 5x5 Gaussian kernel, sigma=0 (auto-calculated based on kernel size)

		// Apply median filtering to further reduce noise
		cv::Mat denoised_depth_image;
		cv::medianBlur(blurred_depth_image, denoised_depth_image, 5); // 5x5 median filter

		// Process depth map (e.g., find distances, detect objects, etc.)
		// Example: find minimum and maximum depth values
		double min_depth, max_depth;
		cv::minMaxLoc(denoised_depth_image, &min_depth, &max_depth);

		// Publish the min and max depth values (as an example)
		auto depth_range_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
		depth_range_msg->data.push_back(min_depth);
		depth_range_msg->data.push_back(max_depth);
		// Publish depth range
		// depth_range_publisher_->publish(*depth_range_msg);
	}

	void updateCameraCalibration(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
	{
		// Convert ROS2 camera info to OpenCV camera matrix and distortion coefficients
		camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
						  msg->k[3], msg->k[4], msg->k[5],
						  msg->k[6], msg->k[7], msg->k[8]);

		distortion_coeffs_ = (cv::Mat_<double>(5, 1) << msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4]);
	}
	void processColorImage(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		// Check if the image is in the expected format
		if (msg->encoding != "rgb8")
		{
			RCLCPP_WARN(this->get_logger(), "Unexpected image encoding: %s", msg->encoding.c_str());
			return;
		}

		// Convert sensor_msgs::Image to cv::Mat
		cv::Mat color_image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()), msg->step);

		// Convert color image to grayscale for marker detection
		cv::Mat gray_image;
		cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);

		// Define dictionary and detector parameters
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

		// Detect ArUco markers
		std::vector<int> marker_ids;
		std::vector<std::vector<cv::Point2f>> marker_corners;
		cv::aruco::detectMarkers(gray_image, dictionary, marker_corners, marker_ids, parameters, cv::noArray(), camera_matrix_, distortion_coeffs_);

		if (marker_ids.empty() || marker_corners.empty())
		{
			RCLCPP_WARN(this->get_logger(), "No markers ids or corners detected");
			return;
		}
		//---------------------------------------------------------------------------------------------------------------
		// // Define the object points (3D coordinates of the marker corners in the marker's coordinate system)
		// std::vector<cv::Point3f> object_points;
		// for (unsigned long int i = 0; i < marker_ids.size(); ++i)
		// {
		// 	object_points.push_back(cv::Point3f(-marker_length / 2, -marker_length / 2, 0));
		// 	object_points.push_back(cv::Point3f(marker_length / 2, -marker_length / 2, 0));
		// 	object_points.push_back(cv::Point3f(marker_length / 2, marker_length / 2, 0));
		// 	object_points.push_back(cv::Point3f(-marker_length / 2, marker_length / 2, 0));
		// }

		// // Convert the detected marker corners to a format suitable for solvePnP
		// std::vector<cv::Point2f> image_points;
		// for (size_t i = 0; i < marker_corners.size(); ++i)
		// {
		// 	for (size_t j = 0; j < marker_corners[i].size(); ++j)
		// 	{
		// 		image_points.push_back(marker_corners[i][j]);
		// 	}
		// }

		// // Estimate pose using solvePnP
		// cv::Mat rvec, tvec;
		// cv::solvePnP(object_points, image_points, camera_matrix_, distortion_coeffs_, rvec, tvec);

		// // Convert rotation vector to quaternion
		// cv::Mat rot_matrix;
		// cv::Rodrigues(rvec, rot_matrix);

		// // Convert rotation matrix to quaternion
		// Eigen::Matrix3d rot_matrix_eigen;
		// cv::cv2eigen(rot_matrix, rot_matrix_eigen);
		// Eigen::Quaterniond quaternion(rot_matrix_eigen);

		// // Publish detected markers and their poses
		// auto pose_array_msg = std::make_shared<geometry_msgs::msg::PoseArray>();
		// pose_array_msg->header = msg->header;

		// for (size_t i = 0; i < marker_ids.size(); ++i)
		// {
		// 	geometry_msgs::msg::Pose pose;
		// 	// Calculate offset to move the origin to the center of the marker
		// 	cv::Vec3d marker_offset(0, 0, 0);
		// 	// Apply offset to translation vector
		// 	cv::Vec3d marker_center = cv::Vec3d(tvec) + marker_offset;
		// 	pose.position.x = marker_center[0];
		// 	pose.position.y = marker_center[1];
		// 	pose.position.z = marker_center[2];

		// 	pose.orientation.x = quaternion.x();
		// 	pose.orientation.y = quaternion.y();
		// 	pose.orientation.z = quaternion.z();
		// 	pose.orientation.w = quaternion.w();

		// 	pose_array_msg->poses.push_back(pose);
		// }
		// marker_publisher_->publish(*pose_array_msg);
		//----------------------------------------------------------------------------------
		// Estimate pose of detected markers using member variables
		std::vector<cv::Vec3d> rvecs, tvecs;
		// Publish detected markers and their poses
		double marker_length = 0.024
		;
		cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, camera_matrix_, distortion_coeffs_, rvecs, tvecs);
		// Check if pose estimation succeeded
		if (rvecs.empty() || tvecs.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Pose estimation failed");
			return;
		}

		auto pose_array_msg = std::make_shared<geometry_msgs::msg::PoseArray>();
		pose_array_msg->header = msg->header;

		for (size_t i = 0; i < marker_ids.size(); ++i)
		{
			geometry_msgs::msg::Pose pose;
			// Calculate offset to move the origin to the center of the marker
			cv::Vec3d marker_offset(0, 0, 0);
			// Apply offset to translation vector
			cv::Vec3d marker_center = tvecs[i] + marker_offset;
			pose.position.x = marker_center[0];
			pose.position.y = marker_center[1];
			pose.position.z = marker_center[2];

			// Convert rotation vector to quaternion
			cv::Mat rot_matrix;
			cv::Rodrigues(rvecs[i], rot_matrix);

			// Convert rotation matrix to quaternion
			Eigen::Matrix3d rot_matrix_eigen;
			cv::cv2eigen(rot_matrix, rot_matrix_eigen);
			Eigen::Quaterniond quaternion(rot_matrix_eigen);

			pose.orientation.x = quaternion.x();
			pose.orientation.y = quaternion.y();
			pose.orientation.z = quaternion.z();
			pose.orientation.w = quaternion.w();

			pose_array_msg->poses.push_back(pose);
		}
		marker_publisher_->publish(*pose_array_msg);
		//----------------------------------------------------------------------------------
		// Calculate to publish the centre pose (Send to Tasman)
		cv::Vec3d avg_translation(0, 0, 0);
		for (size_t i = 0; i < marker_ids.size(); ++i)
		{
			avg_translation += tvecs[i];
		}
		avg_translation /= static_cast<double>(marker_ids.size());

		// Calculate the average orientation quaternion of all markers (assuming equal weights)
		Eigen::Quaterniond avg_quaternion(1.0, 0.0, 0.0, 0.0); // Identity quaternion
		for (size_t i = 0; i < marker_ids.size(); ++i)
		{
			cv::Mat rot_matrix;
			cv::Rodrigues(rvecs[i], rot_matrix);
			Eigen::Matrix3d rot_matrix_eigen;
			cv::cv2eigen(rot_matrix, rot_matrix_eigen);
			Eigen::Quaterniond quaternion(rot_matrix_eigen);
			avg_quaternion = avg_quaternion.slerp(1.0 / (i + 1), quaternion); // Spherical linear interpolation
		}

		// Publish the average center pose
		geometry_msgs::msg::Pose avg_center_pose;
		avg_center_pose.position.x = avg_translation[0];
		avg_center_pose.position.y = avg_translation[1];
		avg_center_pose.position.z = avg_translation[2];
		avg_center_pose.orientation.x = avg_quaternion.x();
		avg_center_pose.orientation.y = avg_quaternion.y();
		avg_center_pose.orientation.z = avg_quaternion.z();
		avg_center_pose.orientation.w = avg_quaternion.w();

		// Create a PoseStamped message to include a header
		auto avg_center_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
		avg_center_pose_msg->header = msg->header;
		avg_center_pose_msg->pose = avg_center_pose;

		// Publish the average center pose
		center_marker_publisher_->publish(*avg_center_pose_msg);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoLocalisationNode>());
	rclcpp::shutdown();
	return 0;
}