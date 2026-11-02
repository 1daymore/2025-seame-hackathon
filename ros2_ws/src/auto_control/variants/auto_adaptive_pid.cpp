#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <numeric>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm> // std::max, std::min
#include <cmath>     // atan2, abs, M_PI

class LaneDetectNode : public rclcpp::Node
    {
    public:
        LaneDetectNode()
        : Node("lane_detect_node"),
        cap_(0),
        throttle_(0.17),
        stopline_detected_(false),
        finish_line_detected_(false),
        is_restarting_(false),
        stop_time_(0),
        restart_time_(0),
        M_computed_(false),
        has_last_(false),
        last_leftx_(80),
        last_rightx_(240),
        last_error_(0.0),
        integral_(0.0),
        stopline_count_(0),
        force_steering_mode_(false),
        forced_steering_start_time_(0.0),
        forced_steering_value_(0.0),
        forced_steering_delay_start_time_(0.0),
        forced_steering_waiting_(false),
        stopline_cooldown_(false),
        stopline_cooldown_start_time_(0.0),
        ignore_stopline_until_(0.0),
        initial_leftx_(0),
        initial_rightx_(0),
        // --- 변경된 부분 시작 ---
        // 초기에는 차선 미감지 시 오른쪽 조향 로직을 비활성화 (false로 시작하여 5초 뒤에 활성화)
        disable_no_lane_steer_(true), // <-- 초기값을 true로 변경 (비활성 상태)
        disable_no_lane_steer_start_time_(0.0) // <-- 이 시간은 두 번째 정지선 감지 시 업데이트될 예정
        // --- 변경된 부분 끝 ---
        {
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "카메라 열기 실패");
                rclcpp::shutdown();
                return;
            }

            cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

            steer_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/auto_steer", 10);
            throttle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/auto_throttle", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&LaneDetectNode::process_frame, this)
            );
            RCLCPP_INFO(this->get_logger(), "LaneDetectNode가 시작되었습니다.");
        }

    private:
        // 함수 선언부 (Declaration)
        void process_frame();
        //void show_image(const cv::Mat& img); // 시각화를 사용하지 않는다면 이 줄은 삭제하거나 계속 주석 처리
        void publish_steer(float steer_value);
        void publish_throttle(float throttle_value);
        bool detect_stopline(const cv::Mat& bev, cv::Mat& vis_img);
        bool detect_checkerboard(const cv::Mat& bev, cv::Mat& vis_img);

    private: // 멤버 변수 정의 (더 나은 가독성을 위해 이곳으로 이동)
        cv::VideoCapture cap_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_publisher_;
        float throttle_;
        
        int stopline_count_;
        bool force_steering_mode_;
        double forced_steering_start_time_;
        float forced_steering_value_;

        double forced_steering_delay_start_time_;
        bool forced_steering_waiting_;
        const double forced_steering_delay_sec_ = 0.0;
        const double forced_steering_duration_sec_ = 4.5;

        bool stopline_cooldown_;
        double stopline_cooldown_start_time_;
        const double stopline_cooldown_duration_ = 2.0;

        // 상태 관리 변수들
        bool stopline_detected_;
        bool finish_line_detected_;
        bool is_restarting_;    
        double stop_time_;
        double restart_time_;
        double ignore_stopline_until_; // 이 시간 전까지는 정지선 감지를 무시
        // 차선 미감지 시 오른쪽 조향 비활성화를 위한 변수 추가
        bool disable_no_lane_steer_; // 이제 이 변수가 true면 '비활성' (오른쪽 조향 안 함), false면 '활성' (오른쪽 조향 가능)
        double disable_no_lane_steer_start_time_;
        const double NO_LANE_STEER_DISABLE_DURATION_ = 12.5; // 5초 비활성화
        // 주의: 변수명은 'DISABLE'로 남아 있지만, 로직상 'true'일 때 비활성화, 'false'일 때 활성화됩니다.
        // 더 명확하게 하려면 'is_no_lane_steer_enabled_ = false;' 로 바꾸고 로직을 반전시키는 것이 좋습니다.

        // 차선 유지를 위한 변수들
        cv::Mat M_;
        bool M_computed_;
        int last_leftx_, last_rightx_;
        bool has_last_;
        const float Kp_ = 0.8;
        const float Ki_ = 0.01;
        const float Kd_ = 0.001;
        float last_error_;
        float integral_;
        const int YELLOW_LANE_PIXEL_THRESHOLD_ = 400; // 노란 차선 우선 인식을 위한 픽셀 임계값

        // 급격한 코너 무시를 위한 변수 추가
        int initial_leftx_;
        int initial_rightx_;
    };


// --- Function Definitions (outside the class definition) ---

void LaneDetectNode::process_frame()
{
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "프레임을 받아오지 못했습니다.");
        return;
    }

    int height = frame.rows;
    int width  = frame.cols;

    // --- 1. BEV 변환 및 이미지 전처리 ---
    if (!M_computed_) {
        std::vector<cv::Point2f> src = {{float(width) * 0.2f, float(height) * 0.5f}, {float(width) * 0.8f, float(height) * 0.5f}, {float(width), float(height)}, {0.0f, float(height)}};
        std::vector<cv::Point2f> dst = {{0.0f, 0.0f}, {float(width), 0.0f}, {float(width), float(height)}, {0.0f, float(height)}};
        M_ = cv::getPerspectiveTransform(src, dst);
        M_computed_ = true;
    }
    cv::Mat bev;
    cv::warpPerspective(frame, bev, M_, cv::Size(width, height));
    // --- [IMPORTANT CHANGE HERE] Keep vis_img as a copy of bev for drawing ---
    cv::Mat vis_img = bev.clone(); 
    
    // [수정] 차선 마스크 생성 로직 변경
    cv::Mat hls, yellow_mask, white_mask, lane_mask, red_mask1, red_mask2, red_mask;
    cv::cvtColor(bev, hls, cv::COLOR_BGR2HLS);

    cv::inRange(hls, cv::Scalar(10, 50, 100), cv::Scalar(40, 255, 255), yellow_mask);
    cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 70), white_mask);

    int yellow_count = cv::countNonZero(yellow_mask);
    int white_count  = cv::countNonZero(white_mask);
    const int YELLOW_LANE_PIXEL_THRESHOLD = 400;
    const int WHITE_LANE_PIXEL_MIN_SWITCH = 1000; // 환경 맞게 조정

    // --- YELLOW PRIORITY LOGIC (as updated in previous response) ---
    if (yellow_count > YELLOW_LANE_PIXEL_THRESHOLD) {
        lane_mask = yellow_mask;
        //RCLCPP_INFO(this->get_logger(), "yellow:%d white:%d mask:YELLOW ONLY (Priority)",
        //            yellow_count, white_count);
    } else {
        // If yellow is not strong enough for priority, then use both yellow and white.
        cv::bitwise_or(yellow_mask, white_mask, lane_mask);
        //RCLCPP_INFO(this->get_logger(), "yellow:%d white:%d mask:YELLOW+WHITE (Default)",
        //              yellow_count, white_count);
    }
    // --- END YELLOW PRIORITY LOGIC ---

    cv::inRange(hls, cv::Scalar(0, 50, 70), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hls, cv::Scalar(170, 50, 70), cv::Scalar(180, 255, 255), red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
        
    cv::Mat masked, gray, blur, edges;
    cv::bitwise_and(bev, bev, masked, lane_mask); 
    cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);
    cv::Canny(blur, edges, 50, 150);

    // --- 2. 차선 탐지 (히스토그램 및 슬라이딩 윈도우) ---
    cv::Mat roi_hist = edges.rowRange(height * 0.6f, height);
    cv::Mat hist_mat;
    cv::reduce(roi_hist, hist_mat, 0, cv::REDUCE_SUM, CV_32S);
    int midpoint = width / 2;
    
    double minVal, maxVal;
    cv::Point minLoc, maxLoc_left, maxLoc_right;
    cv::minMaxLoc(hist_mat.colRange(0, midpoint), &minVal, &maxVal, &minLoc, &maxLoc_left);
    int current_leftx_hist = maxLoc_left.x;
    cv::minMaxLoc(hist_mat.colRange(midpoint, width), &minVal, &maxVal, &minLoc, &maxLoc_right);
    int current_rightx_hist = maxLoc_right.x + midpoint;
    
    int leftx = current_leftx_hist;
    int rightx = current_rightx_hist;

    initial_leftx_ = leftx; 
    initial_rightx_ = rightx;

    last_leftx_ = (last_leftx_ * 0.8f) + (leftx * 0.2f);
    last_rightx_ = (last_rightx_ * 0.8f) + (rightx * 0.2f);
    leftx = static_cast<int>(last_leftx_);
    rightx = static_cast<int>(last_rightx_);

    const int nwindows = 9;
    const int margin = 20;
    const int minpix = 5;
    int window_height = height / nwindows;
    
    std::vector<cv::Point> nonzero_points;
    cv::findNonZero(edges, nonzero_points);
    std::vector<int> nonzerox, nonzeroy;
    nonzerox.reserve(nonzero_points.size());
    nonzeroy.reserve(nonzero_points.size());

    for (const auto& p : nonzero_points) {
        nonzerox.push_back(p.x);
        nonzeroy.push_back(p.y);
    }
    
    for (int w = 0; w < nwindows; ++w) {
        int y_low = height - (w + 1) * window_height;
        int y_high = height - w * window_height;
        int lx_low = leftx - margin, lx_high = leftx + margin;
        int rx_low = rightx - margin, rx_high = rightx + margin;
        
        // --- [IMPORTANT CHANGE HERE] Re-enable drawing sliding windows ---
        cv::rectangle(vis_img, {lx_low, y_low}, {lx_high, y_high}, {255,0,0}, 2);
        cv::rectangle(vis_img, {rx_low, y_low}, {rx_high, y_high}, {0,0,255}, 2);
        
        std::vector<int> good_left_inds, good_right_inds;
        for (size_t i = 0; i < nonzeroy.size(); ++i) {
            int y = nonzeroy[i], x = nonzerox[i];
            if (y >= y_low && y < y_high) {
                if (x >= lx_low && x < lx_high) good_left_inds.push_back(i);
                if (x >= rx_low && x < rx_high) good_right_inds.push_back(i);
            }
        }
        
        if (good_left_inds.size() > minpix) {
            long long sumx = 0; for (auto i : good_left_inds) sumx += nonzerox[i];
            leftx = sumx / good_left_inds.size();
        }
        if (good_right_inds.size() > minpix) {
            long long sumx = 0; for (auto i : good_right_inds) sumx += nonzerox[i];
            rightx = sumx / good_right_inds.size();
        }
    }

    // --- 3. PID 조향 제어 ---
    // --- 3.1. 급격한 코너 무시 로직 추가 ---
    float vertical_dist_for_angle = static_cast<float>(height - (height - nwindows * window_height)); 

    float left_angle_rad = std::atan2(vertical_dist_for_angle, static_cast<float>(leftx - initial_leftx_));
    float right_angle_rad = std::atan2(vertical_dist_for_angle, static_cast<float>(rightx - initial_rightx_));
    
    float left_angle_deg = 90.0f - std::abs(left_angle_rad * 180.0f / M_PI);
    float right_angle_deg = 90.0f - std::abs(right_angle_rad * 180.0f / M_PI);

    if ((left_angle_deg > 65.0 && left_angle_deg < 80.0) || (right_angle_deg > 65.0 && right_angle_deg < 80.0)) {
        RCLCPP_WARN(this->get_logger(), "급격한 코너(L:%.1f, R:%.1f deg) 감지! 이전 차선 정보를 유지합니다.", left_angle_deg, right_angle_deg);
        leftx = last_leftx_; 
        rightx = last_rightx_;
    }

    // PID 조향 계산은 급격한 코너 무시 로직 이후에 와야 합니다.
    last_leftx_ = leftx; 
    last_rightx_ = rightx; 
    has_last_ = true;
    float lane_center = (leftx + rightx) / 2.0f;
    float error = midpoint - lane_center;
    float p_term = Kp_ * error;
    integral_ += error;
    integral_ = std::max(-500.0f, std::min(500.0f, integral_));
    float i_term = Ki_ * integral_;
    float d_term = Kd_ * (error - last_error_);
    last_error_ = error;
    float pid_output = p_term + i_term + d_term;
    float steer_value = -pid_output / (width / 2.0f);
    const float steering_offset = -0.05f;
    steer_value += steering_offset;
    steer_value = std::max(-1.0f, std::min(1.0f, steer_value));
    
    // --- 여기에 강제 조향 로직 추가 ---
    float final_steer_value;
    if (force_steering_mode_) {
        // 강제 조향 모드일 때는 PID 계산 결과 무시하고 강제 값 사용
        final_steer_value = forced_steering_value_;
        //RCLCPP_INFO(this->get_logger(), "강제 조향 중: %.2f", final_steer_value); // 디버깅 로그 추가
    }
    // --- 변경된 부분 시작 ---
    // 차선 미감지 시 오른쪽 조향을 5초 뒤에 활성화하는 로직
    else if (disable_no_lane_steer_) { // 현재 disable_no_lane_steer_가 true (비활성) 상태라면
        if (disable_no_lane_steer_start_time_ != 0.0 && // 시간이 설정되었고
            this->now().seconds() - disable_no_lane_steer_start_time_ > NO_LANE_STEER_DISABLE_DURATION_) {
            // 5초 (NO_LANE_STEER_DISABLE_DURATION_)가 경과했으면
            disable_no_lane_steer_ = false; // 플래그를 false로 변경 (이제 활성 상태)
            RCLCPP_INFO(this->get_logger(), "차선 미감지 시 오른쪽 조향이 활성화되었습니다.");
        }
        // 5초가 경과하지 않았다면, 여전히 비활성 상태이므로 일반 PID 조향을 따릅니다.
        final_steer_value = steer_value; 
    }
    // disable_no_lane_steer_가 false (활성) 상태일 때만 아래 로직 실행
    else if (yellow_count <= 300 && white_count <= 50) {
        // If no lane pixels are detected, turn right
        final_steer_value = 0.7; // Adjust this value as needed for right turn
        RCLCPP_WARN(this->get_logger(), "차선 미감지! 오른쪽으로 조향합니다. Steer: %.2f", final_steer_value);
    }
    // --- 변경된 부분 끝 ---
    else {
        // 일반 주행 모드일 때는 PID 계산 결과 사용
        final_steer_value = steer_value;
    }

    // 최종 조향 값 범위 제한
    final_steer_value = std::max(-1.0f, std::min(1.0f, final_steer_value));
    publish_steer(final_steer_value); // 최종 조향 값 발행

    // --- 4. 스로틀 제어 및 상태 관리 ---
    // --- [IMPORTANT CHANGE HERE] Pass the actual vis_img for drawing ---
    bool raw_is_stopline = detect_stopline(bev, vis_img);
    bool is_checkerboard = detect_checkerboard(bev, vis_img);

    // [추가] 문제 1 해결: 첫 정지선 통과 후 10초간 정지선 감지 무시
    if (this->now().seconds() < ignore_stopline_until_) {
         raw_is_stopline = false; // 쿨다운 시간 동안은 정지선 감지를 비활성화
    }
    
    // 기존의 짧은 쿨다운 (같은 정지선을 여러 번 감지하는 것 방지)
    if (stopline_cooldown_) {
        if (this->now().seconds() - stopline_cooldown_start_time_ > stopline_cooldown_duration_) {
            stopline_cooldown_ = false;
        } else {
            raw_is_stopline = false;
        }
    }
    
    if (finish_line_detected_) {
        throttle_ = 0.0;
    }
    else if (force_steering_mode_) {
        if (this->now().seconds() - forced_steering_start_time_ < forced_steering_duration_sec_) {
            throttle_ = 0.17;
            // steer_value는 publish_steer에서 이미 전달됩니다.
            // 여기서는 강제 조향 중임을 나타내기 위한 로직만 유지
        } else {
            force_steering_mode_ = false;
        }
    }
    else if (forced_steering_waiting_) { // 강제 조향 대기 로직 추가
        if (this->now().seconds() - forced_steering_delay_start_time_ > forced_steering_delay_sec_) {
           // RCLCPP_INFO(this->get_logger(), "강제 조향 시작!");
            forced_steering_waiting_ = false;
            force_steering_mode_ = true;
            forced_steering_start_time_ = this->now().seconds();
            publish_steer(forced_steering_value_); // 대기 후 바로 강제 조향 값 publish
        }
        throttle_ = 0.17; // 대기 중에도 저속 유지
    }
    else if (is_checkerboard) {
        //RCLCPP_INFO(this->get_logger(), "!!! 체커보드 감지! 주행을 종료합니다.");
        //finish_line_detected_ = true;
        throttle_ = 0.17;
    }
    else if (raw_is_stopline) {
        stopline_cooldown_ = true;
        stopline_cooldown_start_time_ = this->now().seconds();
        stopline_count_ += 1;

        //RCLCPP_INFO(this->get_logger(), "정지선 감지! 현재 카운트: %d", stopline_count_);

        if (stopline_count_ == 1) {
            RCLCPP_INFO(this->get_logger(), "!!! 첫 번째 정지선 감지! 3.5초간 정지 및 3초 쿨다운 시작");
            stopline_detected_ = true;
            stop_time_ = this->now().seconds();
            throttle_ = 0.0;
            ignore_stopline_until_ = this->now().seconds() + 3.0;
        }

        if (stopline_count_ == 2) {
            RCLCPP_INFO(this->get_logger(), "!!! 두 번째 정지선 감지! 3.5초간 정지 및 3초 쿨다운 시작");
            stopline_detected_ = true;
            stop_time_ = this->now().seconds();
            throttle_ = 0.0;
            ignore_stopline_until_ = this->now().seconds() + 3.0;

            // --- 변경된 부분 시작 ---
            // 두 번째 정지선 감지 시, 오른쪽 조향 로직 활성화를 위한 타이머 시작
            disable_no_lane_steer_ = true; // <-- 현재는 비활성 (5초 뒤 활성 예정)
            disable_no_lane_steer_start_time_ = this->now().seconds();
            RCLCPP_INFO(this->get_logger(), "두 번째 정지선 감지. 차선 미감지 시 오른쪽 조향 5초 뒤 활성화 예정.");
            // --- 변경된 부분 끝 ---
        }
        // [MODIFIED] Removed the `else if (stopline_count_ == 2)` and `else if (stopline_count_ == 3)` blocks.
    }
    else if (stopline_detected_) {
        if (this->now().seconds() - stop_time_ < 3.5) {
            throttle_ = 0.0;
        } else {
            RCLCPP_INFO(this->get_logger(), ">>> 3.5초 경과. 재출발 시작");
            stopline_detected_ = false;
            is_restarting_ = true; 
            restart_time_ = this->now().seconds();
            throttle_ = 0.17;
        }
    }
    else if (is_restarting_) {
        if (this->now().seconds() - restart_time_ > 3.0) {
            RCLCPP_INFO(this->get_logger(), "유예 기간 종료. 일반 주행으로 전환합니다.");
            is_restarting_ = false;
        }
        throttle_ = 0.17;
    }
    else {
        if (!red_mask.empty() && cv::countNonZero(red_mask) > 800) {
            throttle_ = 0.17;
        } else {
            throttle_ = 0.17;
        }
    }
        
    publish_throttle(throttle_);

    // --- [IMPORTANT CHANGE HERE] Call show_image with the actual vis_img ---
    //show_image(vis_img); 
}

// show_image 함수 정의 제거
/*
void LaneDetectNode::show_image(const cv::Mat& img) {
    cv::imshow("Lane Detection", img);
    cv::waitKey(1);
}*/

void LaneDetectNode::publish_steer(float steer_value) {
    auto msg = std_msgs::msg::Float32();
    msg.data = steer_value;
    steer_publisher_->publish(msg);
}

void LaneDetectNode::publish_throttle(float throttle_value) {
    auto msg = std_msgs::msg::Float32();
    msg.data = throttle_value;
    throttle_publisher_->publish(msg);
}

bool LaneDetectNode::detect_stopline(const cv::Mat& bev, cv::Mat& vis_img)
{
    cv::Rect roi_rect(0, bev.rows - 80, bev.cols, 80);
    cv::Mat roi = bev(roi_rect);
    cv::Mat hls, white_mask, edges;
    cv::cvtColor(roi, hls, cv::COLOR_BGR2HLS);
    cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 255), white_mask);
    cv::Canny(white_mask, edges, 100, 200);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 40, 40, 5);

    double total_horizontal_length = 0;
    for (const auto& line : lines) {
        double angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI;
        if (std::abs(angle) < 10 || std::abs(angle) > 170) {
            total_horizontal_length += std::sqrt(std::pow(line[2] - line[0], 2) + std::pow(line[3] - line[1], 2));
            // --- [IMPORTANT CHANGE HERE] Re-enable drawing stop line ---
            cv::line(vis_img, cv::Point(line[0], line[1] + roi_rect.y), cv::Point(line[2], line[3] + roi_rect.y), cv::Scalar(0, 255, 255), 2);
        }
    }
    return total_horizontal_length > 150.0;
}

bool LaneDetectNode::detect_checkerboard(const cv::Mat& bev, cv::Mat& vis_img)
{
    // const cv::Size board_size(3, 3); // 3x3 체커보드가 아니므로 주석 처리하거나 제거
    cv::Rect roi_rect(0, bev.rows * 0.6, bev.cols, bev.rows * 0.4); // ROI는 트랙 환경에 맞춰 튜닝 필요
    cv::Mat roi = bev(roi_rect);

    cv::Mat hls, white_mask;
    cv::cvtColor(roi, hls, cv::COLOR_BGR2HLS);
    // HLS 범위 튜닝: 조명에 따라 190을 낮출 수 있음 (예: 160)
    cv::inRange(hls, cv::Scalar(0, 160, 0), cv::Scalar(180, 255, 255), white_mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int square_like_contours = 0;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        // 면적 필터링: "아주 작은 크기"에 맞춰 범위 조정
        const double MIN_CHECK_AREA = 5.0;   // 이전 40 -> 5 (더 작은 조각 허용)
        const double MAX_CHECK_AREA = 200.0; // 이전 1000 -> 200 (너무 큰 덩어리 필터링)

        if (area < MIN_CHECK_AREA || area > MAX_CHECK_AREA) continue;

        cv::Rect bbox = cv::boundingRect(contour);
        float aspect_ratio = (float)bbox.width / bbox.height;

        // 종횡비 필터링: "길게 체크보드"이므로 정사각형이 아닐 수 있음. 범위 확장
        const float MIN_ASPECT_RATIO = 0.3; // 이전 0.7 -> 0.3 (더 길거나 납작한 모양 허용)
        const float MAX_ASPECT_RATIO = 3.0; // 이전 1.3 -> 3.0 (더 길거나 납작한 모양 허용)

        if (aspect_ratio > MIN_ASPECT_RATIO && aspect_ratio < MAX_ASPECT_RATIO) {
            square_like_contours++;
            // --- [IMPORTANT CHANGE HERE] Re-enable drawing checkerboard bounding box ---
            cv::Rect full_bbox = bbox + cv::Point(roi_rect.x, roi_rect.y); 
            cv::rectangle(vis_img, full_bbox, cv::Scalar(0, 255, 0), 2); 
        }
    }
    // 최종 판단: "길게" 이어진 패턴이므로 더 많은 조각이 감지될 수 있음. 
    // 튜닝 필요 (예: 5 -> 8 또는 그 이상)
    const int MIN_DETECTED_CHECKS = 8;
    return square_like_contours > MIN_DETECTED_CHECKS;
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}