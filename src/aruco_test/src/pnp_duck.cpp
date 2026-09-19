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

// Localize a single robot by solving PnP on its marker, then transforming the pose into the world frame
// with the static camera TF (world -> camera).
class PnpDuckNode : public rclcpp::Node {
public:
    PnpDuckNode() : Node("pnp_duck_node") {
        this->declare_parameter<std::string>("RGB_topic", "/camera/camera/color/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
        this->declare_parameter<std::string>("pose_topic", "/duck/pose/pnp");
        this->declare_parameter<int>("robot.id", 1);
        this->declare_parameter<double>("robot.marker_size", 0.1);
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
        robot_id_ = this->get_parameter("robot.id").as_int();
        marker_size_ = this->get_parameter("robot.marker_size").as_double();
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
            std::bind(&PnpDuckNode::camera_info_callback, this, std::placeholders::_1));

        RGB_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            RGB_topic_, 10,
            std::bind(&PnpDuckNode::RGB_img_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

        // TODO: compare DICT_4X4_100 / APRILTAG_36h11 / APRILTAG_16h5 accuracy (see TODO.md)
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->polygonalApproxAccuracyRate = 0.05;
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 10;

        // Marker corners in marker frame, order required by SOLVEPNP_IPPE_SQUARE
        // top-left, top-right, bottom-right, bottom-left (same as detectMarkers output)
        const float h = static_cast<float>(marker_size_ / 2.0);
        marker_obj_points_ = {
            cv::Point3f(-h,  h, 0.0f),
            cv::Point3f( h,  h, 0.0f),
            cv::Point3f( h, -h, 0.0f),
            cv::Point3f(-h, -h, 0.0f)
        };
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
            RCLCPP_WARN(this->get_logger(), "Distortion model '%s' is not handled by OpenCV PnP, results may be biased",
                msg->distortion_model.c_str());
        }
    }

    void get_camera_pose() {
        try {
            auto tf_msg = tf_buffer_->lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
            T_world_cam_ = tf2::transformToEigen(tf_msg);
            is_camera_pose_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera position: X=%.3f, Y=%.3f, Z=%.3f",
                T_world_cam_.translation().x(), T_world_cam_.translation().y(), T_world_cam_.translation().z());
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
        }
    }

    static Eigen::Isometry3d cv_pose_to_eigen(const cv::Mat &rvec, const cv::Mat &tvec) {
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                T.linear()(r, c) = R_cv.at<double>(r, c);
            }
            T.translation()(r) = tvec.at<double>(r);
        }
        return T;
    }

    double reprojection_rms(const std::vector<cv::Point2f> &img_points, const cv::Mat &rvec, const cv::Mat &tvec) const {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(marker_obj_points_, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
        double sum_sq = 0.0;
        for (size_t i = 0; i < projected.size(); i++) {
            const cv::Point2f d = projected[i] - img_points[i];
            sum_sq += d.x * d.x + d.y * d.y;
        }
        return std::sqrt(sum_sq / projected.size());
    }

    void RGB_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!is_camera_info_received_ || !is_camera_pose_initialized_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for camera info and camera position...");
            if (is_camera_info_received_) {
                get_camera_pose();
            }
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

        int target_idx = -1;
        for (size_t i = 0; i < marker_ids.size(); i++) {
            if (marker_ids[i] == robot_id_) {
                target_idx = static_cast<int>(i);
                break;
            }
        }

        double raw_pose[3] = {0.0, 0.0, 0.0};
        double final_pose[3] = {0.0, 0.0, 0.0};
        double yaw_rad = 0.0;
        double yaw_deg = 0.0;
        double err_best = 0.0;
        double err_other = -1.0;
        cv::Mat rvec_best, tvec_best;
        bool is_pose_valid = false;

        if (target_idx >= 0) {
            const auto &corners = marker_corners[target_idx];
            std::vector<cv::Mat> rvecs, tvecs;
            const int n_solutions = cv::solvePnPGeneric(
                marker_obj_points_, corners, camera_matrix_, dist_coeffs_, rvecs, tvecs,
                false, cv::SOLVEPNP_IPPE_SQUARE);

            // IPPE gives two candidate poses for a planar square (flip ambiguity).
            // Overhead camera: pick the one whose marker normal (+z) points closest to world +Z.
            int best = -1;
            double best_up = -2.0;
            Eigen::Isometry3d T_world_marker_best;
            for (int k = 0; k < n_solutions; k++) {
                const Eigen::Isometry3d T_world_marker = T_world_cam_ * cv_pose_to_eigen(rvecs[k], tvecs[k]);
                const double up = T_world_marker.linear()(2, 2);
                if (up > best_up) {
                    best_up = up;
                    best = k;
                    T_world_marker_best = T_world_marker;
                }
            }

            if (best >= 0) {
                rvec_best = rvecs[best];
                tvec_best = tvecs[best];
                err_best = reprojection_rms(corners, rvec_best, tvec_best);
                if (n_solutions > 1) {
                    err_other = reprojection_rms(corners, rvecs[1 - best], tvecs[1 - best]);
                }

                raw_pose[0] = T_world_marker_best.translation().x();
                raw_pose[1] = T_world_marker_best.translation().y();
                raw_pose[2] = T_world_marker_best.translation().z();
                pose_filter(raw_pose, final_pose);

                // yaw from marker x-axis (left -> right) projected on the world XY plane
                const Eigen::Matrix3d &R = T_world_marker_best.linear();
                yaw_rad = std::atan2(R(1, 0), R(0, 0));
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
                    "Target 3D raw:(%.3f, %.3f, %.3f) filtered:(%.3f, %.3f, %.3f), Yaw: %.3f rad %.3f deg, "
                    "reproj err: %.3f px (other solution: %.3f px)",
                    raw_pose[0], raw_pose[1], raw_pose[2],
                    final_pose[0], final_pose[1], final_pose[2],
                    yaw_rad, yaw_deg,
                    err_best, err_other
                );
                // broadcast TF for debugging
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = msg->header.stamp;
                t.header.frame_id = world_frame_;
                t.child_frame_id = "pnp_duck_" + std::to_string(robot_id_);
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
                if (is_pose_valid) {
                    cv::drawFrameAxes(RGB_frame, camera_matrix_, dist_coeffs_, rvec_best, tvec_best,
                                      static_cast<float>(marker_size_));
                }
                cv::imshow("PnP Duck", RGB_frame);
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
    Eigen::Isometry3d T_world_cam_ = Eigen::Isometry3d::Identity();
    std::vector<cv::Point3f> marker_obj_points_;

    int robot_id_ = 1;
    double marker_size_ = 0.1;

    bool is_debug_mode_ = false;
    bool image_debug_ = false;

    bool is_camera_info_received_ = false;
    bool is_camera_pose_initialized_ = false;
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
    auto node = std::make_shared<PnpDuckNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
