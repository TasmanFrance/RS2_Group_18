#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

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
			"/camera/camera/depth/camera_info",
			10,
			[this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
			{
				// Update camera matrix and distortion coefficients
				updateCameraCalibration(msg);
			});
		marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
			"detected_markers",
			10);
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_publisher_;

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

		// Process depth map (e.g., find distances, detect objects, etc.)
		// Example: find minimum and maximum depth values
		double min_depth, max_depth;
		cv::minMaxLoc(depth_image, &min_depth, &max_depth);

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

		// Detect ArUco markers
		std::vector<int> marker_ids;
		std::vector<std::vector<cv::Point2f>> marker_corners;
		cv::aruco::detectMarkers(gray_image, dictionary, marker_corners, marker_ids, parameters, cv::noArray(), camera_matrix_, distortion_coeffs_);

		if (marker_ids.empty())
		{
			RCLCPP_WARN(this->get_logger(), "No markers detected");
			return;
		}

		// Estimate pose of detected markers using member variables
		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.14, camera_matrix_, distortion_coeffs_, rvecs, tvecs);

		// Check if pose estimation succeeded
		if (rvecs.empty() || tvecs.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Pose estimation failed");
			return;
		}

		// Publish detected markers and their poses
		auto pose_array_msg = std::make_shared<geometry_msgs::msg::PoseArray>();
		pose_array_msg->header = msg->header;

		for (size_t i = 0; i < marker_ids.size(); ++i)
		{
			geometry_msgs::msg::Pose pose;
			pose.position.x = tvecs[i][0];
			pose.position.y = tvecs[i][1];
			pose.position.z = tvecs[i][2];

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
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoLocalisationNode>());
	rclcpp::shutdown();
	return 0;
}
