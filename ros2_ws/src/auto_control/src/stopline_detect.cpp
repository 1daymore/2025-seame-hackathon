#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>

class LaneDetectSubscriber : public rclcpp::Node
{
public:
    LaneDetectSubscriber()
    : Node("image_subscriber"),
      has_last_(false),
      M_computed_(false)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video_frames", 10,
            std::bind(&LaneDetectSubscriber::listener_callback, this, std::placeholders::_1)
        );
    }

private:
    void listener_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // --- 1) Convert to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image;
        int height = frame.rows;
        int width  = frame.cols;

        // --- 2) Compute/persist BEV transform once
        if (!M_computed_) {
            std::vector<cv::Point2f> src = {
                {width * 0.3f, height * 0.4f},
                {width * 0.7f, height * 0.4f},
                {float(width), float(height)},
                {0.0f,       float(height)}
            };
            std::vector<cv::Point2f> dst = {
                {0.0f,       0.0f},
                {float(width), 0.0f},
                {float(width), float(height)},
                {0.0f,       float(height)}
            };
            M_ = cv::getPerspectiveTransform(src, dst);
            M_computed_ = true;
        }

        // --- 3) Apply Bird's Eye View
        cv::Mat bev;
        cv::warpPerspective(frame, bev, M_, cv::Size(width, height));

        // --- 4) Color thresholding (HLS)
        cv::Mat hls, yellow_mask, white_mask, combined_mask;
        cv::cvtColor(bev, hls, cv::COLOR_BGR2HLS);
        cv::inRange(hls, cv::Scalar(10, 50, 100), cv::Scalar(40, 255, 255), yellow_mask);
        cv::inRange(hls, cv::Scalar(0, 200,   0), cv::Scalar(180, 255,  70), white_mask);
        cv::bitwise_or(yellow_mask, white_mask, combined_mask);

        // --- 5) Mask & edge detection
        cv::Mat masked, gray, blur, edges;
        cv::bitwise_and(bev, bev, masked, combined_mask);
        cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);
        cv::Canny(blur, edges, 50, 150);

        // --- 6) Histogram over bottom 50% of image
        cv::Mat roi = edges.rowRange(height / 2, height);
        cv::Mat hist_mat;
        cv::reduce(roi, hist_mat, 0, cv::REDUCE_SUM, CV_32S);
        std::vector<int> hist;
        hist_mat.colRange(0, hist_mat.cols).copyTo(hist);

        int midpoint = width / 2;
        int leftx  = std::distance(hist.begin(),
                        std::max_element(hist.begin(), hist.begin() + midpoint));
        int rightx = std::distance(hist.begin() + midpoint,
                        std::max_element(hist.begin() + midpoint, hist.end()))
                     + midpoint;

        int sum_left  = std::accumulate(hist.begin(), hist.begin() + midpoint, 0);
        int sum_right = std::accumulate(hist.begin() + midpoint, hist.end(), 0);

        // --- 7) Fallback with OR condition and python-style defaults
        if (leftx == 0 || sum_left  < 5000) {
            leftx = has_last_ ? last_leftx_ : (width / 4);
        }
        if (rightx == midpoint || sum_right < 5000) {
            rightx = has_last_ ? last_rightx_ : (width * 3 / 4);
        }

        // --- 8) Sliding-window refinement
        const int nwindows      = 15;
        const int margin        = 30;
        const int minpix        = 5;
        int window_height       = height / nwindows;

        // collect non-zero edge points
        std::vector<int> nonzeroy, nonzerox;
        for (int y = 0; y < edges.rows; ++y) {
            for (int x = 0; x < edges.cols; ++x) {
                if (edges.at<uchar>(y,x) > 0) {
                    nonzeroy.push_back(y);
                    nonzerox.push_back(x);
                }
            }
        }

        cv::Mat window_img = cv::Mat::zeros(bev.size(), bev.type());
        for (int w = 0; w < nwindows; ++w) {
            int y_low  = height - (w + 1) * window_height;
            int y_high = height - w * window_height;

            int lx_low  = leftx  - margin;
            int lx_high = leftx  + margin;
            int rx_low  = rightx - margin;
            int rx_high = rightx + margin;

            cv::rectangle(window_img, {lx_low, y_low}, {lx_high, y_high}, {255,0,0}, 2);
            cv::rectangle(window_img, {rx_low, y_low}, {rx_high, y_high}, {0,0,255}, 2);

            std::vector<int> good_left_inds, good_right_inds;
            for (size_t i = 0; i < nonzeroy.size(); ++i) {
                int x = nonzerox[i], y = nonzeroy[i];
                if (y >= y_low && y < y_high) {
                    if (x >= lx_low && x < lx_high)  good_left_inds.push_back(i);
                    if (x >= rx_low && x < rx_high)  good_right_inds.push_back(i);
                }
            }
            if (good_left_inds.size()  > minpix) {
                int sumx = 0; for (auto i : good_left_inds)  sumx += nonzerox[i];
                leftx  = sumx / static_cast<int>(good_left_inds.size());
            }
            if (good_right_inds.size() > minpix) {
                int sumx = 0; for (auto i : good_right_inds) sumx += nonzerox[i];
                rightx = sumx / static_cast<int>(good_right_inds.size());
            }
        }
        

        // --- 9) Overlay and display
        cv::Mat overlay;
        cv::addWeighted(bev, 1.0, window_img, 0.3, 0.0, overlay);

        bool stopline_found = detect_stopline(bev, overlay);
        if (stopline_found) {
            RCLCPP_INFO(this->get_logger(), "🚦 정지선 감지됨!");
        }
        cv::imshow("Lane Detection (with BEV + Sliding Window)", overlay);
        cv::waitKey(1);

        // --- 10) Save for next frame
        last_leftx_  = leftx;
        last_rightx_ = rightx;
        has_last_    = true;
    }

    bool detect_stopline(const cv::Mat& bev, cv::Mat& vis)  // 👈 여기 추가
    {
        int height = bev.rows;
        int width = bev.cols;

        cv::Rect roi_rect(0, height - 60, width, 60);
        cv::Mat roi = bev(roi_rect);

        cv::Mat mask;
        cv::inRange(roi, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool stopline_found = false;
        for (const auto& contour : contours) {
            cv::Rect bbox = cv::boundingRect(contour);
            float aspect_ratio = (float)bbox.width / bbox.height;

            if (aspect_ratio > 3.0 && bbox.height > 5 && bbox.width > 80) {
                stopline_found = true;
                cv::Rect full_bbox = bbox + cv::Point(roi_rect.x, roi_rect.y);
                cv::rectangle(vis, full_bbox, cv::Scalar(0, 255, 255), 2);
            }
        }
        return stopline_found;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Mat       M_;
    bool          M_computed_;
    int           last_leftx_, last_rightx_;
    bool          has_last_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectSubscriber>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}