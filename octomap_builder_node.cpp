#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OctomapBuilder
{
public:
    OctomapBuilder()
    : nh_(), private_nh_("~"), camera_model_initialized_(false), octree_(0.1),
      tf_buffer_(), tf_listener_(tf_buffer_)
    {
        info_sub_ = nh_.subscribe("/depth_cam/camera_info", 1, &OctomapBuilder::cameraInfoCallback, this);
        depth_sub_ = nh_.subscribe("/depth_cam/image_raw", 1, &OctomapBuilder::depthImageCallback, this);

        private_nh_.param<std::string>("world_frame", world_frame_, "world");

        ROS_INFO("Octomap Builder Node Initialized with world frame: %s", world_frame_.c_str());
    }

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber depth_sub_, info_sub_;
    image_geometry::PinholeCameraModel cam_model_;
    bool camera_model_initialized_;
    octomap::OcTree octree_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string world_frame_;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        cam_model_.fromCameraInfo(info_msg);
        camera_model_initialized_ = true;
        ROS_INFO_ONCE("Camera model initialized from /camera_info.");
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
    {
        if (!camera_model_initialized_) return;

        ros::Time lookup_time = depth_msg->header.stamp - ros::Duration(0.001);
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                world_frame_, depth_msg->header.frame_id,
                lookup_time,
                ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF lookup failed: %s", ex.what());
            return;
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        const std::string& encoding = depth_msg->encoding;
        cv::Mat depth_img;

        if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_img = cv_ptr->image;  // 单位：米
        } else if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_ptr->image.convertTo(depth_img, CV_32FC1, 1.0 / 1000.0);  // 毫米 → 米
            ROS_INFO_ONCE("Converted depth image from 16UC1 to 32FC1 (meters)");
        } else {
            ROS_ERROR("Unsupported depth encoding: %s", encoding.c_str());
            return;
        }     

        //cv_bridge::CvImagePtr cv_ptr;
        //try {
        //    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        //} catch (cv_bridge::Exception& e) {
        //    ROS_ERROR("cv_bridge exception: %s", e.what());
        //    return;
        //}

        //const cv::Mat& depth_img = cv_ptr->image;
        int width = depth_img.cols;
        int height = depth_img.rows;

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                float d = depth_img.at<float>(v, u);
                if (std::isnan(d) || d <= 0.1 || d > 5.0) continue;

                // 相机坐标系下的点
                cv::Point2d pixel(u, v);
                cv::Point3d ray = cam_model_.projectPixelTo3dRay(pixel);
                ray *= d;

                // 构造 geometry_msgs 点用于 tf 变换
                geometry_msgs::PointStamped cam_point, world_point;
                cam_point.header.frame_id = depth_msg->header.frame_id;
                cam_point.header.stamp = depth_msg->header.stamp;
                cam_point.point.x = ray.x;
                cam_point.point.y = ray.y;
                cam_point.point.z = ray.z;

                try {
                    tf2::doTransform(cam_point, world_point, transform);
                } catch (tf2::TransformException& ex) {
                    ROS_WARN("TF point transform failed: %s", ex.what());
                    continue;
                }

                // 插入变换后的点到 Octomap
                octomap::point3d endpoint(world_point.point.x,
                                          world_point.point.y,
                                          world_point.point.z);
                octree_.updateNode(endpoint, true); // 占据
            }
        }

        octree_.updateInnerOccupancy();
        octree_.writeBinary("/tmp/octomap.bt");

        //ROS_INFO("Octomap updated with TF and saved to /tmp/octomap.bt");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_builder_node");
    OctomapBuilder builder;
    ros::spin();
    return 0;
}
