#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

// Localize a single robot with a ground-plane homography.
// No field markers: H is derived from the static camera TF (world -> camera) and camera intrinsics.
class HomographyDuckNode : public rclcpp::Node {
public:
    HomographyDuckNode() : Node("homography_duck_node") {
        this->declare_parameter<std::string>("RGB_topic", "/camera/camera/color/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
        this->declare_parameter<std::string>("pose_topic", "/duck/pose/homography");
        this->declare_parameter<double>("target_height", 0.447);
        this->declare_parameter<int>("robot.id", 1);
        this->declare_parameter<std::string>("world_frame", "map");
        this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
        this->declare_parameter<bool>("pose_filter.enable", false);
        this->declare_parameter<double>("pose_filter.alpha", 0.1);
        this->declare_parameter<double>("pose_filter.max_jump_m", 0.15);
        this->declare_parameter<bool>("debug.enable", false);
        this->declare_parameter<bool>("debug.img", false);
        RGB_topic_ = this->get_parameter("RGB_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        pose_topic_ = this->get_parameter("pose_topic").as_string();
        target_height_ = this->get_parameter("target_height").as_double();
        robot_id_ = this->get_parameter("robot.id").as_int();
        world_frame_ = this->get_parameter("world_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        pose_filter_enable_ = this->get_parameter("pose_filter.enable").as_bool();
        pose_filter_alpha_ = this->get_parameter("pose_filter.alpha").as_double();
        pose_filter_max_jump_m_ = this->get_parameter("pose_filter.max_jump_m").as_double();
        is_debug_mode_ = this->get_parameter("debug.enable").as_bool();
        image_debug_ = this->get_parameter("debug.img").as_bool();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 10,
            std::bind(&HomographyDuckNode::camera_info_callback, this, std::placeholders::_1));

        RGB_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            RGB_topic_, 10,
            std::bind(&HomographyDuckNode::RGB_img_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

        // TODO: compare DICT_4X4_100 / APRILTAG_36h11 / APRILTAG_16h5 accuracy (see TODO.md)
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->polygonalApproxAccuracyRate = 0.05;
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 10;
    }

private:
    void pose_filter(const double raw_pose[3], double filtered_pose[3]) {
        filtered_pose[0] = raw_pose[0];
        filtered_pose[1] = raw_pose[1];
        filtered_pose[2] = raw_pose[2];

        if (!pose_filter_enable_) {
            pose_filter_initialized_ = false;
            return;
        }

        // keep filter tunable without restart (alpha/max_jump can be updated at runtime)
        pose_filter_alpha_ = this->get_parameter("pose_filter.alpha").as_double();
        pose_filter_max_jump_m_ = this->get_parameter("pose_filter.max_jump_m").as_double();
        const double alpha = std::clamp(pose_filter_alpha_, 0.0, 1.0);

        if (!pose_filter_initialized_) {
            pose_filtered_[0] = raw_pose[0];
            pose_filtered_[1] = raw_pose[1];
            pose_filtered_[2] = raw_pose[2];
            pose_filter_initialized_ = true;
        } else {
            const double dx = raw_pose[0] - pose_filtered_[0];
            const double dy = raw_pose[1] - pose_filtered_[1];
            const double dz = raw_pose[2] - pose_filtered_[2];
            const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (pose_filter_max_jump_m_ > 0.0 && dist > pose_filter_max_jump_m_) {
                // outlier guard: snap to measurement on big jumps
                pose_filtered_[0] = raw_pose[0];
                pose_filtered_[1] = raw_pose[1];
                pose_filtered_[2] = raw_pose[2];
            } else {
                pose_filtered_[0] = alpha * raw_pose[0] + (1.0 - alpha) * pose_filtered_[0];
                pose_filtered_[1] = alpha * raw_pose[1] + (1.0 - alpha) * pose_filtered_[1];
                pose_filtered_[2] = alpha * raw_pose[2] + (1.0 - alpha) * pose_filtered_[2];
            }
        }

        filtered_pose[0] = pose_filtered_[0];
        filtered_pose[1] = pose_filtered_[1];
        filtered_pose[2] = pose_filtered_[2];
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (is_camera_info_received_) {
            return;
        }
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) {
            camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
        }
        dist_coeffs_ = cv::Mat(msg->d, true).reshape(1, 1);
        is_camera_info_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Camera info received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f, distortion_model=%s",
            msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->distortion_model.c_str());
        if (!msg->d.empty() && msg->distortion_model != "plumb_bob" && msg->distortion_model != "rational_polynomial") {
            RCLCPP_WARN(this->get_logger(), "Distortion model '%s' is not handled by cv::undistortPoints, results may be biased",
                msg->distortion_model.c_str());
        }
    }

    // Build the image -> ground(z=0) homography from camera extrinsics and intrinsics
    void init_ground_homography() {
        if (!is_camera_info_received_) {
            return;
        }
        try {
            auto tf_msg = tf_buffer_->lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
            const Eigen::Isometry3d T_world_cam = tf2::transformToEigen(tf_msg);

            cam_x_ = T_world_cam.translation().x();
            cam_y_ = T_world_cam.translation().y();
            cam_z_ = T_world_cam.translation().z();

            // world -> camera
            const Eigen::Matrix3d R_cw = T_world_cam.rotation().transpose();
            const Eigen::Vector3d t_cw = -R_cw * T_world_cam.translation();

            // For points on z=0: s * [u, v, 1]^T = K * [r1 r2 t] * [X, Y, 1]^T
            cv::Mat Rt = (cv::Mat_<double>(3, 3) <<
                R_cw(0, 0), R_cw(0, 1), t_cw(0),
                R_cw(1, 0), R_cw(1, 1), t_cw(1),
                R_cw(2, 0), R_cw(2, 1), t_cw(2));
            H_world_to_img_ = camera_matrix_ * Rt;
            H_ = H_world_to_img_.inv();
            H_ /= H_.at<double>(2, 2);

            is_camera_position_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera position: X=%.3f, Y=%.3f, Z=%.3f", cam_x_, cam_y_, cam_z_);
            RCLCPP_INFO(this->get_logger(), "Ground homography (image -> world):\n[%.6f, %.6f, %.6f]\n[%.6f, %.6f, %.6f]\n[%.6f, %.6f, %.6f]",
                H_.at<double>(0, 0), H_.at<double>(0, 1), H_.at<double>(0, 2),
                H_.at<double>(1, 0), H_.at<double>(1, 1), H_.at<double>(1, 2),
                H_.at<double>(2, 0), H_.at<double>(2, 1), H_.at<double>(2, 2));
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
        }
    }

    // Apply H to a pixel, return false if the result is at infinity
    bool pixel_to_ground(const cv::Point2f &px, double &X, double &Y) const {
        cv::Mat pt_src = (cv::Mat_<double>(3, 1) << px.x, px.y, 1.0);
        cv::Mat pt_dst = H_ * pt_src;
        const double w = pt_dst.at<double>(2, 0);
        if (std::abs(w) < 1e-12) {
            return false;
        }
        X = pt_dst.at<double>(0, 0) / w;
        Y = pt_dst.at<double>(1, 0) / w;
        return true;
    }

    // Draw world origin and 1 m X/Y axes projected on the ground, for checking extrinsics by eye
    void draw_world_axes(cv::Mat &img) const {
        auto project = [this](double X, double Y, cv::Point &out) {
            cv::Mat p = H_world_to_img_ * (cv::Mat_<double>(3, 1) << X, Y, 1.0);
            const double w = p.at<double>(2, 0);
            if (w <= 1e-9) {
                return false;  // behind camera
            }
            out = cv::Point(cvRound(p.at<double>(0, 0) / w), cvRound(p.at<double>(1, 0) / w));
            return true;
        };
        cv::Point o, px, py;
        if (!project(0.0, 0.0, o)) {
            return;
        }
        if (project(1.0, 0.0, px)) {
            cv::line(img, o, px, cv::Scalar(0, 0, 255), 2);
            cv::putText(img, "X", px, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
        if (project(0.0, 1.0, py)) {
            cv::line(img, o, py, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, "Y", py, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }

    void RGB_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!is_camera_position_initialized_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for camera info and camera position...");
            init_ground_homography();
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat RGB_frame = cv_ptr->image;

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        cv::aruco::detectMarkers(RGB_frame, dictionary_, marker_corners, marker_ids, detector_params_, rejected_candidates);

        std::vector<cv::Point2f> target_corners;
        bool is_target_found = false;
        for (size_t i = 0; i < marker_ids.size(); i++) {
            if (marker_ids[i] == robot_id_) {
                // H is built for an ideal pinhole camera, so remove lens distortion first
                cv::undistortPoints(marker_corners[i], target_corners, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
                is_target_found = true;
                break;
            }
        }

        double raw_pose[3] = {0.0, 0.0, 0.0};
        double final_pose[3] = {0.0, 0.0, 0.0};
        double yaw_rad = 0.0;
        double yaw_deg = 0.0;
        cv::Point2f target_center(0.0f, 0.0f);
        bool is_pose_valid = false;

        if (is_target_found && target_corners.size() == 4) {
            // tl, tr, br, bl
            const cv::Point2f &tl = target_corners[0];
            const cv::Point2f &tr = target_corners[1];
            const cv::Point2f &br = target_corners[2];
            const cv::Point2f &bl = target_corners[3];
            target_center = (tl + br) * 0.5f;

            double X_g, Y_g, X_left, Y_left, X_right, Y_right;
            if (pixel_to_ground(target_center, X_g, Y_g) &&
                pixel_to_ground((bl + tl) * 0.5f, X_left, Y_left) &&
                pixel_to_ground((br + tr) * 0.5f, X_right, Y_right)) {
                // project target pixel to Z=0 ground to get shadow, then
                // use 3D similar triangle linear interpolation to get Z=target_height_ coordinates
                const double t = (cam_z_ - target_height_) / cam_z_;
                raw_pose[0] = cam_x_ + t * (X_g - cam_x_);
                raw_pose[1] = cam_y_ + t * (Y_g - cam_y_);
                raw_pose[2] = target_height_;
                pose_filter(raw_pose, final_pose);

                // yaw from marker left -> right direction on the ground
                // (scaling about the camera center keeps the direction, so no height correction needed)
                yaw_rad = std::atan2(Y_right - Y_left, X_right - X_left);
                yaw_deg = yaw_rad * 180.0 / CV_PI;

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = msg->header.stamp;
                pose_msg.header.frame_id = world_frame_;
                pose_msg.pose.position.x = final_pose[0];
                pose_msg.pose.position.y = final_pose[1];
                pose_msg.pose.position.z = final_pose[2];

                // roll = pitch = 0, yaw = yaw_rad
                const double half_yaw = yaw_rad * 0.5;
                pose_msg.pose.orientation.x = 0.0;
                pose_msg.pose.orientation.y = 0.0;
                pose_msg.pose.orientation.z = std::sin(half_yaw);
                pose_msg.pose.orientation.w = std::cos(half_yaw);

                pose_pub_->publish(pose_msg);
                is_pose_valid = true;
            }
        }

        // ==========================================
        // Debug
        // ==========================================
        if (is_debug_mode_) {
            if (is_pose_valid) {
                RCLCPP_INFO(this->get_logger(),
                    "Target 3D raw:(%.3f, %.3f, %.3f) filtered:(%.3f, %.3f, %.3f), Yaw: %.3f rad %.3f deg",
                    raw_pose[0], raw_pose[1], raw_pose[2],
                    final_pose[0], final_pose[1], final_pose[2],
                    yaw_rad, yaw_deg
                );
                // broadcast TF for debugging
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = msg->header.stamp;
                t.header.frame_id = world_frame_;
                t.child_frame_id = "homo_duck_" + std::to_string(robot_id_);
                t.transform.translation.x = final_pose[0];
                t.transform.translation.y = final_pose[1];
                t.transform.translation.z = final_pose[2];
                const double half_yaw = yaw_rad * 0.5;
                t.transform.rotation.x = 0.0;
                t.transform.rotation.y = 0.0;
                t.transform.rotation.z = std::sin(half_yaw);
                t.transform.rotation.w = std::cos(half_yaw);
                tf_broadcaster_->sendTransform(t);
            }

            if (image_debug_) {
                if (!marker_ids.empty()) {
                    cv::aruco::drawDetectedMarkers(RGB_frame, marker_corners, marker_ids);
                }
                // draw rejected candidates (purple) for debugging
                if (!rejected_candidates.empty()) {
                    cv::aruco::drawDetectedMarkers(RGB_frame, rejected_candidates, cv::noArray(), cv::Scalar(255, 0, 255));
                }
                draw_world_axes(RGB_frame);
                // draw a red point on the target center (undistorted pixel)
                if (is_target_found) {
                    cv::circle(RGB_frame, target_center, 5, cv::Scalar(0, 0, 255), -1);
                }
                cv::imshow("Homography Duck", RGB_frame);
                cv::waitKey(1);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr RGB_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::string RGB_topic_;
    std::string camera_info_topic_;
    std::string pose_topic_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string world_frame_;
    std::string camera_frame_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat H_;               // image -> world ground (z=0)
    cv::Mat H_world_to_img_;  // world ground (z=0) -> image

    double cam_x_ = 0.0, cam_y_ = 0.0, cam_z_ = 0.0;
    double target_height_;

    int robot_id_ = 1;

    bool is_debug_mode_ = false;
    bool image_debug_ = false;

    bool is_camera_info_received_ = false;
    bool is_camera_position_initialized_ = false;
    bool pose_filter_enable_ = true;
    bool pose_filter_initialized_ = false;
    double pose_filter_alpha_ = 0.2;
    double pose_filter_max_jump_m_ = 0.5;
    double pose_filtered_[3] = {0.0, 0.0, 0.0};

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HomographyDuckNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
