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
      throttle_(0.2),
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
      initial_leftx_(0), // 초기화 추가
      initial_rightx_(0) // 초기화 추가
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
    // 함수 선언부 (Declaration) - 여기에만 둡니다.
    void process_frame();
    void show_image(const cv::Mat& img); // 시각화 함수 제거
    void publish_steer(float steer_value);
    void publish_throttle(float throttle_value);
    bool detect_stopline(const cv::Mat& bev, cv::Mat& vis_img);
    bool detect_checkerboard(const cv::Mat& bev, cv::Mat& vis_img);

private: // Moved member variables here for better organization
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
    const double forced_steering_delay_sec_ = 0.3;
    const double forced_steering_duration_sec_ = 1.3;

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

    // 차선 유지를 위한 변수들
    cv::Mat M_;
    bool M_computed_;
    int last_leftx_, last_rightx_;
    bool has_last_;
    const float Kp_ = 0.65;
    const float Ki_ = 0.008;
    const float Kd_ = 0.0008;
    float last_error_;
    float integral_;
    const int YELLOW_LANE_PIXEL_THRESHOLD_ = 30; // 노란 차선 우선 인식을 위한 픽셀 임계값

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
        std::vector<cv::Point2f> src = {{float(width) * 0.2f, float(height) * 0.4f}, {float(width) * 0.8f, float(height) * 0.4f}, {float(width), float(height)}, {0.0f, float(height)}};
        std::vector<cv::Point2f> dst = {{0.0f, 0.0f}, {float(width), 0.0f}, {float(width), float(height)}, {0.0f, float(height)}};
        M_ = cv::getPerspectiveTransform(src, dst);
        M_computed_ = true;
    }
    cv::Mat bev;
    cv::warpPerspective(frame, bev, M_, cv::Size(width, height));
    cv::Mat vis_img = bev.clone(); // 시각화 변수 제거
    
    // [수정] 차선 마스크 생성 로직 변경
    cv::Mat hls, yellow_mask, white_mask, lane_mask, red_mask1, red_mask2, red_mask;
    cv::cvtColor(bev, hls, cv::COLOR_BGR2HLS);

    // 노란색, 흰색 마스크 각각 생성
    cv::inRange(hls, cv::Scalar(15, 80, 70), cv::Scalar(35, 255, 255), yellow_mask);
    cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 70), white_mask);

    // [추가] 문제 2 해결: 노란색 차선 우선 인식
    if (cv::countNonZero(yellow_mask) > YELLOW_LANE_PIXEL_THRESHOLD_) {
        lane_mask = yellow_mask; // 노란색 픽셀이 충분하면 노란색 마스크만 사용
    } else {
        cv::bitwise_or(yellow_mask, white_mask, lane_mask); // 그렇지 않으면 둘을 합쳐서 사용
    }

    cv::inRange(hls, cv::Scalar(0, 50, 70), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hls, cv::Scalar(170, 50, 70), cv::Scalar(180, 255, 255), red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    
    cv::Mat masked, gray, blur, edges;
    cv::bitwise_and(bev, bev, masked, lane_mask); // [수정] combined_mask 대신 lane_mask 사용
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

    // 슬라이딩 윈도우가 시작되는 이미지 하단 영역의 차선 X좌표를 initial_leftx, initial_rightx로 사용
    // 이 부분은 히스토그램에서 찾은 초기 차선 위치를 그대로 사용할 수 있습니다.
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
        
        cv::rectangle(vis_img, {lx_low, y_low}, {lx_high, y_high}, {255,0,0}, 2); // 시각화 코드 제거
        cv::rectangle(vis_img, {rx_low, y_low}, {rx_high, y_high}, {0,0,255}, 2); // 시각화 코드 제거
        
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
    // vertical_dist는 BEV 이미지의 높이를 기준으로 차선의 기울기를 계산할 기준 Y축 거리입니다.
    // 현재 슬라이딩 윈도우의 맨 위 차선 X좌표와 맨 아래(initial) 차선 X좌표를 사용하여 기울기를 계산합니다.
    float vertical_dist_for_angle = static_cast<float>(height - (height - nwindows * window_height)); // 맨 아래 윈도우 시작점부터 맨 위 윈도우 끝점까지의 수직 거리
                                                                                                    // 또는 단순하게 전체 BEV 이미지 높이를 사용해도 됩니다 (height).
                                                                                                    // 여기서는 슬라이딩 윈도우가 적용되는 전체 높이를 사용했습니다.

    // 슬라이딩 윈도우의 가장 상단 (가장 먼) 윈도우의 leftx, rightx를 최종 leftx, rightx로 사용
    // initial_leftx_, initial_rightx_는 맨 아래 윈도우의 시작점 (히스토그램 기반)
    float left_angle_rad = std::atan2(vertical_dist_for_angle, static_cast<float>(leftx - initial_leftx_));
    float right_angle_rad = std::atan2(vertical_dist_for_angle, static_cast<float>(rightx - initial_rightx_));
    
    // 90도를 기준으로 차선이 수직에서 얼마나 벗어났는지 계산 (0도에 가까울수록 수평)
    float left_angle_deg = 90.0f - std::abs(left_angle_rad * 180.0f / M_PI);
    float right_angle_deg = 90.0f - std::abs(right_angle_rad * 180.0f / M_PI);

    // 급격한 코너 판단 기준 (예: 70도 ~ 80도)
    if ((left_angle_deg > 75.0 && left_angle_deg < 80.0) || (right_angle_deg > 75.0 && right_angle_deg < 80.0)) {
        RCLCPP_WARN(this->get_logger(), "급격한 코너(L:%.1f, R:%.1f deg) 감지! 이전 차선 정보를 유지합니다.", left_angle_deg, right_angle_deg);
        // 급격한 코너일 경우, 현재 계산된 leftx, rightx 대신 이전에 사용된 last_leftx_, last_rightx_를 사용
        // 이는 PID 제어 입력에 영향을 줍니다.
        leftx = last_leftx_; 
        rightx = last_rightx_;
    }


    // PID 조향 계산은 급격한 코너 무시 로직 이후에 와야 합니다.
    last_leftx_ = leftx; // 업데이트된 (또는 이전 것으로 유지된) leftx 저장
    last_rightx_ = rightx; // 업데이트된 (또는 이전 것으로 유지된) rightx 저장
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
    publish_steer(steer_value);

    // --- 4. 스로틀 제어 및 상태 관리 ---
    // detect_stopline과 detect_checkerboard는 vis_img를 파라미터로 받지만,
    // 해당 함수들 내부에서 vis_img에 그림을 그리는 시각화 로직이 없다면,
    // 이 파라미터는 그냥 전달해도 무방합니다. 만약 내부에도 시각화 코드가 있다면
    // 해당 부분을 제거해야 합니다. (아래 함수 정의에서 제거 완료)
    cv::Mat dummy_vis_img; // 시각화가 필요 없으므로 더미 Mat 생성
    bool raw_is_stopline = detect_stopline(bev, dummy_vis_img);
    bool is_checkerboard = detect_checkerboard(bev, dummy_vis_img);

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
            throttle_ = 0.18;
            publish_steer(forced_steering_value_); // 대기 후 바로 강제 조향 값 publish
        } else {
            force_steering_mode_ = false;
        }
    }
    else if (forced_steering_waiting_) { // 강제 조향 대기 로직 추가
        if (this->now().seconds() - forced_steering_delay_start_time_ > forced_steering_delay_sec_) {
            RCLCPP_INFO(this->get_logger(), "강제 조향 시작!");
            forced_steering_waiting_ = false;
            force_steering_mode_ = true;
            forced_steering_start_time_ = this->now().seconds();
            }
        throttle_ = 0.18; // 대기 중에도 저속 유지
    }
    else if (is_checkerboard) {
        RCLCPP_INFO(this->get_logger(), "!!! 체커보드 감지! 주행을 종료합니다.");
        finish_line_detected_ = true;
        throttle_ = 0.0;
    }
    else if (raw_is_stopline) {
        stopline_cooldown_ = true;
        stopline_cooldown_start_time_ = this->now().seconds();
        stopline_count_ += 1;

        RCLCPP_INFO(this->get_logger(), "정지선 감지! 현재 카운트: %d", stopline_count_);

        if (stopline_count_ == 1) {
            RCLCPP_INFO(this->get_logger(), "!!! 첫 번째 정지선 감지! 3.5초간 정지 및 7.5초 쿨다운 시작");
            stopline_detected_ = true;
            stop_time_ = this->now().seconds();
            throttle_ = 0.0;
            ignore_stopline_until_ = this->now().seconds() + 7.5;
        }
        else if (stopline_count_ == 2) {
            RCLCPP_INFO(this->get_logger(), "!!! 두 번째 정지선 감지 → 좌회전 강제 조향 대기 시작!");
            forced_steering_waiting_ = true;
            forced_steering_delay_start_time_ = this->now().seconds();
            forced_steering_value_ = -0.4;
            throttle_ = 0.18;
        }
        else if (stopline_count_ == 3) {
            RCLCPP_INFO(this->get_logger(), "!!! 세 번째 정지선 감지 → 우회전 강제 조향 시작!");
            forced_steering_waiting_ = true;
            forced_steering_delay_start_time_ = this->now().seconds();
            forced_steering_value_ = 0.0;
            throttle_ = 0.18;
        }
    }
    else if (stopline_detected_) {
        if (this->now().seconds() - stop_time_ < 3.5) {
            throttle_ = 0.0;
        } else {
            RCLCPP_INFO(this->get_logger(), ">>> 3.5초 경과. 재출발 시작");
            stopline_detected_ = false;
            is_restarting_ = true; 
            restart_time_ = this->now().seconds();
            throttle_ = 0.18;
        }
    }
    else if (is_restarting_) {
        if (this->now().seconds() - restart_time_ > 30.0) {
            RCLCPP_INFO(this->get_logger(), "유예 기간 종료. 일반 주행으로 전환합니다.");
            is_restarting_ = false;
        }
        throttle_ = 0.18;
    }
    else {
        if (!red_mask.empty() && cv::countNonZero(red_mask) > 800) {
            throttle_ = 0.18;
        } else {
            throttle_ = 0.18;
        }
    }
        
    publish_throttle(throttle_);
}

// show_image 함수 정의 제거

void LaneDetectNode::show_image(const cv::Mat& img) {
    cv::imshow("Lane Detection", img);
    cv::waitKey(1);
}


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
            cv::line(vis_img, cv::Point(line[0], line[1] + roi_rect.y), cv::Point(line[2], line[3] + roi_rect.y), cv::Scalar(0, 255, 255), 2); // 시각화 코드 제거
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
            // cv::Rect full_bbox = bbox + cv::Point(roi_rect.x, roi_rect.y); // 시각화 코드 제거
            // cv::rectangle(vis_img, full_bbox, cv::Scalar(0, 255, 0), 2); // 시각화 코드 제거
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