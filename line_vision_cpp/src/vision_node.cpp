#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SensorDataQoS(),  // ใช้แทน qos_profile_sensor_data
            std::bind(&VisionNode::image_callback, this, std::placeholders::_1)
        );
        error_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/line_error",
            10
        );
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        int w = frame.cols;
        int h = frame.rows;

        int roi_top = static_cast<int>(h * 0.75);  // roi = region of interest
        cv::Mat roi = frame(cv::Rect(0, roi_top, w, h - roi_top));

        cv::Mat hsv;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower(100, 100, 50);
        cv::Scalar upper(130, 255, 255);
        cv::Mat mask;
        cv::inRange(hsv, lower, upper, mask);

        cv::Moments M = cv::moments(mask);

        cv::rectangle(frame, cv::Point(0, roi_top), cv::Point(w - 1, h - 1),
            cv::Scalar(255, 0, 0), 2);
        cv::line(frame, cv::Point(w / 2, roi_top), cv::Point(w / 2, h),
            cv::Scalar(0, 255, 0), 2);

        if (M.m00 != 0.0) {
            int cx = static_cast<int>(M.m10 / M.m00);
            int cy = static_cast<int>(M.m01 / M.m00);
            int offset = cx - (w / 2);
            double error = offset / (w / 2.0);
            error = std::clamp(error, -1.0, 1.0);

            std_msgs::msg::Float32 error_msg;
            error_msg.data = static_cast<float>(error);
            error_publisher_->publish(error_msg);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "center=(%d,%d), offset=%d, error=%.3f", cx, cy, offset, error);

            cv::circle(frame, cv::Point(cx, cy + roi_top), 6,
                cv::Scalar(0, 0, 255), -1);
            cv::putText(frame, "offset=" + std::to_string(offset), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "not seen line");
        }

        cv::imshow("Camera", frame);
        cv::imshow("Mask", mask);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();

    rclcpp::spin(node);

    cv::destroyAllWindows();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
