#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

// Per-frame timing columns; must match the Python node so results are comparable.
static const char * kTimingHeader = "convert_us,process_us,publish_us,debug_us,total_us,found";

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        // show_debug: draw overlay + imshow windows (off for benchmarking)
        // timing_csv: if set, write per-frame stage timings there on shutdown
        // zero_copy:  toCvShare (no copy, read-only) instead of toCvCopy
        show_debug_ = this->declare_parameter("show_debug", true);
        timing_csv_ = this->declare_parameter("timing_csv", std::string(""));
        zero_copy_ = this->declare_parameter("zero_copy", false);

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

    void write_timing() const {
        if (timing_csv_.empty()) {
            return;
        }
        std::ofstream out(timing_csv_);
        out << kTimingHeader << "\n";
        for (const auto & row : timing_rows_) {
            out << row << "\n";
        }
    }

private:
    using Clock = std::chrono::steady_clock;

    static double us_between(Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::micro>(b - a).count();
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        const auto t0 = Clock::now();
        cv::Mat frame;
        try {
            if (zero_copy_) {
                frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            } else {
                frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            }
        }
        catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        const auto t1 = Clock::now();

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

        const bool found = M.m00 != 0.0;
        int cx = 0;
        int cy = 0;
        int offset = 0;
        double error = 0.0;
        if (found) {
            cx = static_cast<int>(M.m10 / M.m00);
            cy = static_cast<int>(M.m01 / M.m00);
            offset = cx - (w / 2);
            error = offset / (w / 2.0);
            error = std::clamp(error, -1.0, 1.0);
        }
        const auto t2 = Clock::now();

        if (found) {
            std_msgs::msg::Float32 error_msg;
            error_msg.data = static_cast<float>(error);
            error_publisher_->publish(error_msg);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "center=(%d,%d), offset=%d, error=%.3f", cx, cy, offset, error);
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "not seen line");
        }
        const auto t3 = Clock::now();

        if (show_debug_) {
            // toCvShare gives a read-only view of the ROS message; draw on a copy.
            cv::Mat canvas = zero_copy_ ? frame.clone() : frame;
            cv::rectangle(canvas, cv::Point(0, roi_top), cv::Point(w - 1, h - 1),
                cv::Scalar(255, 0, 0), 2);
            cv::line(canvas, cv::Point(w / 2, roi_top), cv::Point(w / 2, h),
                cv::Scalar(0, 255, 0), 2);
            if (found) {
                cv::circle(canvas, cv::Point(cx, cy + roi_top), 6,
                    cv::Scalar(0, 0, 255), -1);
                cv::putText(canvas, "offset=" + std::to_string(offset), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
            cv::imshow("Camera", canvas);
            cv::imshow("Mask", mask);
            cv::waitKey(1);
        }
        const auto t4 = Clock::now();

        if (!timing_csv_.empty()) {
            char row[128];
            std::snprintf(row, sizeof(row), "%.1f,%.1f,%.1f,%.1f,%.1f,%d",
                us_between(t0, t1), us_between(t1, t2), us_between(t2, t3),
                us_between(t3, t4), us_between(t0, t4), found ? 1 : 0);
            timing_rows_.emplace_back(row);
        }
    }

    bool show_debug_;
    std::string timing_csv_;
    bool zero_copy_;
    std::vector<std::string> timing_rows_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();

    rclcpp::spin(node);

    node->write_timing();
    cv::destroyAllWindows();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
