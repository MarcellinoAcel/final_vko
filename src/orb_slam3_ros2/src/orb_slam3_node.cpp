#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "orb_slam3_ros2/utility.hpp"
using namespace std;
class ORB_SLAM3Node : public rclcpp::Node
{
public:
    ORB_SLAM3Node(std::shared_ptr<ORB_SLAM3::System> pSLAM, const rclcpp::NodeOptions &options)
        : Node("ORB_SLAM3_ROS2", options),
          m_SLAM(pSLAM)
    {
        // Create subscribers with this node
        rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
            this, "/camera/camera/color/image_raw");
        depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
            this, "/camera/camera/depth/image_rect_raw");

        syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
            approximate_sync_policy(10), *rgb_sub, *depth_sub);
        syncApproximate->registerCallback(&ORB_SLAM3Node::GrabRGBD, this);
    }

    ~ORB_SLAM3Node()
    {
        // Reset subscribers and sync first
        syncApproximate.reset();
        rgb_sub.reset();
        depth_sub.reset();
    }

    // Factory method to create shared pointer
    static std::shared_ptr<ORB_SLAM3Node> create(std::shared_ptr<ORB_SLAM3::System> pSLAM, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    {
        auto node = std::make_shared<ORB_SLAM3Node>(pSLAM, options);
        // Additional initialization if needed
        return node;
    }

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD)
    {
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Copy the ros depth image message to cv::Mat.
        try
        {
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    }

    std::shared_ptr<ORB_SLAM3::System> m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
};

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // Create SLAM system
    bool visualization = true;
    auto slam = std::make_shared<ORB_SLAM3::System>(
        argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    auto node = ORB_SLAM3Node::create(slam);

    // Use MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    std::cout << "Spinning with MultiThreadedExecutor..." << std::endl;
    executor.spin();

    if (slam) {
        slam->Shutdown();
    }

    node.reset();
    slam.reset();

    rclcpp::shutdown();
    return 0;
}