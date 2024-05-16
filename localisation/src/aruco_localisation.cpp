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
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

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
		marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
			"detected_markers",
			10);
		// center_marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		// 	"centerpage_marker",
		// 	10);
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_publisher_;
	// rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_marker_publisher_;

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
	//--------------------------------------------------------------------------
	// TESTING IN PROGRESS
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

		// Create ChArUco board
		cv::Ptr<cv::aruco::CharucoBoard> charuco_board = cv::aruco::CharucoBoard::create(5, 5, 0.032, 0.024, dictionary);

		// Detect ChArUco markers
		std::vector<int> marker_ids;
		std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
		cv::aruco::detectMarkers(gray_image, dictionary, marker_corners, marker_ids, parameters, rejected_candidates, camera_matrix_, distortion_coeffs_);

		// Check if markers are detected
		if (marker_corners.empty() || marker_ids.empty())
		{
			RCLCPP_WARN(this->get_logger(), "No markers detected");
			return;
		}
		
		// Detect ChArUco board corners
		std::vector<cv::Point2f> charuco_corners;
		std::vector<int> charuco_ids;
		cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray_image, charuco_board, charuco_corners, charuco_ids, camera_matrix_, distortion_coeffs_);

		// Check if ChArUco corners are detected
		if (charuco_corners.empty() || charuco_ids.empty())
		{
			RCLCPP_WARN(this->get_logger(), "No ChArUco corners detected");
			return;
		}

		// Estimate pose of ChArUco board
		cv::Vec3d rvec, tvec;
		bool pose_estimated = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, charuco_board, camera_matrix_, distortion_coeffs_, rvec, tvec);

		// Check if pose estimation succeeded
		if (!pose_estimated)
		{
			RCLCPP_WARN(this->get_logger(), "Pose estimation failed");
			return;
		}

		auto pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
		pose_stamped_msg->header = msg->header;

		// Apply offset to translation vector
		cv::Vec3d marker_center = tvec;
		pose_stamped_msg->pose.position.x = marker_center[0];
		pose_stamped_msg->pose.position.y = marker_center[1];
		pose_stamped_msg->pose.position.z = marker_center[2];

		// Convert rotation vector to quaternion
		cv::Mat rot_matrix;
		cv::Rodrigues(rvec, rot_matrix);

		// Convert rotation matrix to quaternion
		Eigen::Matrix3d rot_matrix_eigen;
		cv::cv2eigen(rot_matrix, rot_matrix_eigen);
		Eigen::Quaterniond quaternion(rot_matrix_eigen);

		pose_stamped_msg->pose.orientation.x = quaternion.x();
		pose_stamped_msg->pose.orientation.y = quaternion.y();
		pose_stamped_msg->pose.orientation.z = quaternion.z();
		pose_stamped_msg->pose.orientation.w = quaternion.w();

		marker_publisher_->publish(*pose_stamped_msg);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoLocalisationNode>());
	rclcpp::shutdown();
	return 0;
}