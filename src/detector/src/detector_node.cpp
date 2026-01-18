#include "detector/detector_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace exchange_slot
{
ExchangeSlotDetectorNode::ExchangeSlotDetectorNode(const rclcpp::NodeOptions & options)
: Node("exchange_slot_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing ExchangeSlotDetectorNode!");

  detector_ = std::make_unique<ExchangeSlotDetector>();

  // 1. 参数与 Marker 初始化
  this->declare_parameter("square_size", 0.25);
  this->declare_parameter("debug", true);
  this->declare_parameter("binary_thresh", 150);
  this->declare_parameter("no_hardware", true);

  double square_size = this->get_parameter("square_size").as_double();
  debug_ = this->get_parameter("debug").as_bool();
  no_hardware_ = this->get_parameter("no_hardware").as_bool();


  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 初始化底座 Marker (正方形)
  base_marker_.ns = "slot_base";
  base_marker_.type = visualization_msgs::msg::Marker::CUBE;
  base_marker_.scale.x = square_size;
  base_marker_.scale.y = square_size;
  base_marker_.scale.z = 0.01; // 薄片
  base_marker_.color.r = 0.0; base_marker_.color.g = 1.0; base_marker_.color.b = 0.0; base_marker_.color.a = 0.5;
  base_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
  
  // 初始化圆柱体 Marker
  cylinder_marker_.ns = "slot_cylinder";
  cylinder_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
  cylinder_marker_.scale.x = 0.05; // 半径
  cylinder_marker_.scale.y = 0.05;
  cylinder_marker_.scale.z = 0.20; // 高度
  cylinder_marker_.color.r = 1.0; cylinder_marker_.color.g = 0.0; cylinder_marker_.color.b = 0.0; cylinder_marker_.color.a = 0.8;
  cylinder_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 2. 发布者与订阅者
  target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/exchange_slot/target_pose", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/exchange_slot/marker", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (debug_) createDebugPublishers();

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cv::Mat k(3, 3, CV_64F, const_cast<double*>(camera_info->k.data()));
      cv::Mat d(1, 5, CV_64F, const_cast<double*>(camera_info->d.data()));
      detector_->setCameraParams(k.clone(), d.clone());
      cam_info_sub_.reset(); 
    });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ExchangeSlotDetectorNode::imageCallback, this, std::placeholders::_1));
}

void ExchangeSlotDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // 1. 获取图像
  auto cv_ptr = cv_bridge::toCvShare(img_msg, "rgb8");
  cv::Mat img = cv_ptr->image.clone(); 

  // 2. 更新检测参数
  detector_->setParams(this->get_parameter("binary_thresh").as_int(), 
                       this->get_parameter("square_size").as_double(), debug_);

  // 3. 执行检测逻辑
  bool detected = detector_->detect(img);

  // 4. 计算时延
  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000.0;

  if (detected) {
    auto pose_data = detector_->getLastPose();
    
    // 先构造相对于相机的位姿 (Camera Frame)
    geometry_msgs::msg::PoseStamped pose_in_camera;
    pose_in_camera.header = img_msg->header; // 此时 frame_id 是 camera_optical_frame
    pose_in_camera.pose.position.x = pose_data.tvec[0];
    pose_in_camera.pose.position.y = pose_data.tvec[1];
    pose_in_camera.pose.position.z = pose_data.tvec[2];

    cv::Mat rot_mat;
    cv::Rodrigues(pose_data.rvec, rot_mat);
    tf2::Matrix3x3 tf2_rot(
        rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
        rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
        rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
    );
    tf2::Quaternion q;
    tf2_rot.getRotation(q);
    pose_in_camera.pose.orientation = tf2::toMsg(q);
    try {
        // 统一转到 base_link（机械臂底座）
        geometry_msgs::msg::PoseStamped pose_in_world = tf_buffer_->transform(pose_in_camera, "base_link");
        
        target_pose_pub_->publish(pose_in_world);

        if (debug_) {
          // 直接传pose_in_world 给 marker
          publishMarkers(pose_in_world.pose, pose_in_world.header);
          detector_->drawResults(img);
          
          // 发布 TF
          geometry_msgs::msg::TransformStamped t;
          t.header = pose_in_world.header;
          t.child_frame_id = "target_slot_fixed";
          t.transform.translation.x = pose_in_world.pose.position.x;
          t.transform.translation.y = pose_in_world.pose.position.y;
          t.transform.translation.z = pose_in_world.pose.position.z;
          t.transform.rotation = pose_in_world.pose.orientation;
          tf_broadcaster_->sendTransform(t);
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(get_logger(), "Transform failed: %s", ex.what());
      }
  }
  // 图像发布
  if (debug_) {
    std::stringstream ss;
    ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    if (!detected) ss << " | TARGET LOST"; 
    
    cv::Scalar text_color = detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(img, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);
    
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }
}

void ExchangeSlotDetectorNode::publishMarkers(const geometry_msgs::msg::Pose & pose, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray ma;

  // 输入的 pose 是 world/base_link 坐标系下的
  base_marker_.header = header; 
  base_marker_.pose = pose;
  base_marker_.id = 0;

  cylinder_marker_.header = header;
  
  // 计算圆柱体相对于底座的偏移
  tf2::Transform tf_base, tf_offset, tf_cylinder;
  tf2::fromMsg(pose, tf_base);
  tf_offset.setOrigin(tf2::Vector3(0, 0, 0.1)); // 向上偏移 10cm
  tf_offset.setRotation(tf2::Quaternion::getIdentity());
  
  tf_cylinder = tf_base * tf_offset;
  tf2::toMsg(tf_cylinder, cylinder_marker_.pose);
  cylinder_marker_.id = 1;

  ma.markers.push_back(base_marker_);
  ma.markers.push_back(cylinder_marker_);
  marker_pub_->publish(ma);
}

void ExchangeSlotDetectorNode::createDebugPublishers()
{
  binary_img_pub_ = image_transport::create_publisher(this, "/exchange_slot/binary_img");
  result_img_pub_ = image_transport::create_publisher(this, "/exchange_slot/result_img");
}

void ExchangeSlotDetectorNode::destroyDebugPublishers()
{
  binary_img_pub_.shutdown();
  result_img_pub_.shutdown();
}
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(exchange_slot::ExchangeSlotDetectorNode)