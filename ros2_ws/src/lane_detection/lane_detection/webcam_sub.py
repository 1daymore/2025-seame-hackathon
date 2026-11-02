# lane_detect_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.last_leftx = None
        self.last_rightx = None

                # ✅ 해상도 설정
        self.target_width = 640
        self.target_height = 480

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ✅ Bird's Eye View 변환
        bev = apply_birds_eye_view(frame)

        # ✅ 차선 검출
        lane_img, (leftx, rightx) = detect_lane(
            bev,
            self.last_leftx,
            self.last_rightx
        )

        self.last_leftx = leftx
        self.last_rightx = rightx

        cv2.imshow("Lane Detection (with BEV + Sliding Window)", lane_img)
        cv2.waitKey(1)

def apply_birds_eye_view(image):
    """Bird’s Eye View 투시 변환"""
    height, width = image.shape[:2]
    src = np.float32([
        [width * 0.3, height * 0.4],
        [width * 0.7, height * 0.4],
        [width, height],
        [0, height]
    ])
    dst = np.float32([
        [0, 0],
        [width, 0],
        [width, height],
        [0, height]
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, M, (width, height))

def detect_lane(image, last_leftx=None, last_rightx=None):
    height, width = image.shape[:2]

    # HLS 마스크
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    yellow_mask = cv2.inRange(hls, np.array([10, 50, 100]), np.array([40, 255, 255]))
    white_mask = cv2.inRange(hls, np.array([0, 200, 0]), np.array([180, 255, 70]))
    combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
    masked = cv2.bitwise_and(image, image, mask=combined_mask)

    # 엣지
    gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # 히스토그램 기반 초기 위치
    histogram = np.sum(edges[height//2:, :], axis=0)
    midpoint = width // 2
    leftx = np.argmax(histogram[:midpoint])
    rightx = np.argmax(histogram[midpoint:]) + midpoint

    # 보정
    if leftx == 0 or np.sum(histogram[:midpoint]) < 5000:
        leftx = last_leftx if last_leftx else width // 4
    if rightx == midpoint or np.sum(histogram[midpoint:]) < 5000:
        rightx = last_rightx if last_rightx else width * 3 // 4

    # 슬라이딩 윈도우
    nwindows = 15
    margin = 30
    minpix = 5
    window_height = height // nwindows
    nonzeroy, nonzerox = np.nonzero(edges)
    window_img = np.zeros_like(image)
    left_lane_inds, right_lane_inds = [], []

    for window in range(nwindows):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height

        left_win_xlow = leftx - margin
        left_win_xhigh = leftx + margin
        right_win_xlow = rightx - margin
        right_win_xhigh = rightx + margin

        cv2.rectangle(window_img, (left_win_xlow, win_y_low), (left_win_xhigh, win_y_high), (255, 0, 0), 2)
        cv2.rectangle(window_img, (right_win_xlow, win_y_low), (right_win_xhigh, win_y_high), (0, 0, 255), 2)

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= left_win_xlow) & (nonzerox < left_win_xhigh)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= right_win_xlow) & (nonzerox < right_win_xhigh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx = int(np.mean(nonzerox[good_right_inds]))

    overlay = cv2.addWeighted(image, 1, window_img, 0.3, 0)
    return overlay, (leftx, rightx)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
