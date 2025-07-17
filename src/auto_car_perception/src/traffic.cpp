#include "traffic.h"

ColorDetectionNode::ColorDetectionNode()
    : rgb_received(false)
{
    ros::NodeHandle nh;

    rgb_camera_sub = nh.subscribe(
        "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1,
        &ColorDetectionNode::rgbCameraCallback, this);

    semantic_camera_sub = nh.subscribe(
        "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1,
        &ColorDetectionNode::semanticCameraCallback, this);

    depth_camera_sub = nh.subscribe(
        "/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1,
        &ColorDetectionNode::depthCameraCallback, this);

    traffic_light_presence_pub = nh.advertise<std_msgs::Bool>("/traffic_light_presence", 10);

    result_pub = nh.advertise<sensor_msgs::Image>("/Traffic_Debug", 1);
    stop_pub = nh.advertise<std_msgs::Bool>("/stop", 10);
    vehicle_control_pub = nh.advertise<simulation::VehicleControl>("/car_command", 10);
    
    detection_areas.push_back({-64.0, -61.0, -13.0, -10.0});
    detection_areas.push_back({225.0, 230.0, 11.0, 20.0});
    detection_areas.push_back({133.0, 140.0, 2.0, 6.0});
    detection_areas.push_back({42.0, 47.0, 9.0, 17.0});
    detection_areas.push_back({-50.0, -39.0, -3.0, 4.0});

    pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &ColorDetectionNode::poseCallback, this);
}

// RGB相机图像回调
void ColorDetectionNode::rgbCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        rgb_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        rgb_received = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error (RGB): %s", e.what());
    }
}

// 深度图像回调
void ColorDetectionNode::depthCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        depth_received = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error (depth): %s", e.what());
    }
}

// 检测区域回调
void ColorDetectionNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    in_detection_area = false;
    for (const auto& area : detection_areas) {
        if (area.contains(x, y)) {
            in_detection_area = true;
            break;
        }
    }

    ROS_INFO_THROTTLE(5.0, "Current pos: (%.2f, %.2f), in area: %s",
                      x, y, in_detection_area ? "YES" : "NO");
}

//语义相机图像回调
void ColorDetectionNode::semanticCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!rgb_received) {
        // ROS_WARN("No RGB image received yet, skipping semantic callback.");
        return;
    }

    cv::Mat semantic_image;
    try {
        semantic_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error (semantic): %s", e.what());
        return;
    }

    // resize到和RGB一致
    cv::resize(semantic_image, semantic_image, rgb_image.size());

    cv::Mat hsv_image;
    cv::cvtColor(semantic_image, hsv_image, cv::COLOR_BGR2HSV);

    // 黄色阈值
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 发布有红绿灯的信息
    std_msgs::Bool light_presence_msg;
    light_presence_msg.data = !contours.empty();
    traffic_light_presence_pub.publish(light_presence_msg);

    if (!in_detection_area) {
        ROS_INFO_THROTTLE(5.0, "Not in detection area, skipping traffic light detection logic.");
        return;
    }

    ROS_INFO("Found %lu Traffic Light(s)", contours.size());

    if (contours.empty()) {
        std_msgs::Bool stop_msg; stop_msg.data = false; stop_pub.publish(stop_msg);
        return;
    }

    // 按中心距离排序
    double cx = rgb_image.cols / 2.0;

    std::sort(contours.begin(), contours.end(), [cx](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        auto ra = cv::boundingRect(a);
        auto rb = cv::boundingRect(b);

        double center_ax = ra.x + ra.width / 2.0;
        double center_bx = rb.x + rb.width / 2.0;

        return std::abs(center_ax - cx) < std::abs(center_bx - cx);
    });

    auto contour = contours.front();
    auto rect = cv::boundingRect(contour);

    cv::Mat debug_img = rgb_image.clone();
    cv::rectangle(debug_img, rect, cv::Scalar(0, 255, 0), 2);

    int pad = 0;
    int x = std::max(0, rect.x - pad);
    int y = std::max(0, rect.y - pad);
    int w = std::min(rect.width + 2 * pad, rgb_image.cols - x);
    int h = std::min(rect.height + 2 * pad, rgb_image.rows - y);

    if (w > 0 && h > 0)
    {
        cv::Mat roi = rgb_image(cv::Rect(x, y, w, h));
        detectColorsInROI(roi);
    }

    sensor_msgs::Image debug_msg;
    debug_msg = *cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
    result_pub.publish(debug_msg);
}

// ROI 内检测红/绿/黄
void ColorDetectionNode::detectColorsInROI(const cv::Mat& roi)
{
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar red_low1(0, 150, 180), red_high1(5, 255, 255);
    cv::Scalar red_low2(170, 150, 180), red_high2(180, 255, 255);
    cv::Scalar green_low(50, 100, 100), green_high(85, 255, 255);
    cv::Scalar yellow_low(22, 160, 200), yellow_high(30, 255, 255);
    cv::Scalar all_low(0, 0, 0), all_high(255, 255, 255);

    cv::Mat mask_red1, mask_red2, mask_red, mask_green, mask_yellow, mask_all;

    cv::inRange(hsv, red_low1, red_high1, mask_red1);
    cv::inRange(hsv, red_low2, red_high2, mask_red2);
    mask_red = mask_red1 | mask_red2;

    cv::inRange(hsv, green_low, green_high, mask_green);
    cv::inRange(hsv, yellow_low, yellow_high, mask_yellow);
    cv::inRange(hsv, all_low, all_high, mask_all);

    double area_total = cv::sum(mask_all)[0];
    double area_red = cv::sum(mask_red)[0];
    double area_green = cv::sum(mask_green)[0];
    double area_yellow = cv::sum(mask_yellow)[0];

    double threshold = area_total / 50.0;

    std_msgs::Bool stop_msg;

    if (area_red > threshold) {
        ROS_INFO("Detected RED");
        stop_msg.data = true;
        stop_pub.publish(stop_msg);
    } else if (area_green > threshold) {
        ROS_INFO("Detected GREEN");
        stop_msg.data = false;
        stop_pub.publish(stop_msg);
    } else if (area_yellow > threshold) {
        ROS_INFO("Detected YELLOW");
        stop_msg.data = true;
        stop_pub.publish(stop_msg);
    } else {
        ROS_INFO("No signal detected");
        stop_msg.data = false;
        stop_pub.publish(stop_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detection_node");
    ColorDetectionNode node;
    ros::spin();
    return 0;
}