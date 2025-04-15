#include "camera_hld.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

constexpr const char* DEFAULT_NODE_NAME = "camera_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB = "raw_image";
constexpr const char* DEFAULT_TOPIC_NAME_PUB = "face_info";
constexpr const char* TF_CAMERA_FRAME_ID_PARAMETER = "tf_frame_id";
constexpr const char* DEFAULT_TF_CAMERA_FRAME_ID = "camera";
constexpr const char* DEBUG_TOPIC_NAME_PUB = "debug_image";

constexpr double FACE_WIDTH_CM = 20.0;
constexpr double FOCAL_LENGTH_PIXEL = 487.50;


namespace camera_hld {

CameraHLD::CameraHLD() 
: rclcpp::Node(DEFAULT_NODE_NAME),
  raw_image_sub_(create_subscription<sensor_msgs::msg::Image>(
            DEFAULT_TOPIC_NAME_SUB, 10, std::bind(&CameraHLD::imageCallback, this, std::placeholders::_1))),
  debug_image_pub_(create_publisher<sensor_msgs::msg::Image>(DEBUG_TOPIC_NAME_PUB, 10))

{

  tf_frame_id_ = this->declare_parameter<std::string>(TF_CAMERA_FRAME_ID_PARAMETER, DEFAULT_TF_CAMERA_FRAME_ID);
  if(tf_frame_id_== DEFAULT_TF_CAMERA_FRAME_ID) {
    RCLCPP_INFO(this->get_logger(), "Default value for the parameter '%s' will be used for node '%s'",TF_CAMERA_FRAME_ID_PARAMETER, this->get_name());
  }

  face_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(DEFAULT_TOPIC_NAME_PUB, 10);

  face_detector_.load_model(CLFML_FACE_DETECTOR_CPU_MODEL_PATH);

  RCLCPP_INFO(this->get_logger(), "Starting CameraHLD with node name: '%s' and '%s': '%s' ", this->get_name(), TF_CAMERA_FRAME_ID_PARAMETER, tf_frame_id_.c_str());
}

/*virtual*/ CameraHLD::~CameraHLD()
{
}

void CameraHLD::imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{  
  // Convert the ROS 2 image message to an OpenCV image
  cv::Mat frame = convertImageMsgToCvMat(image_msg);
  
  if(frame.empty()) {
    RCLCPP_INFO(this->get_logger(), "Received an empty image frame.");
    return; // no need to to processing on empty image.
  }
  
  face_detector_.load_image(frame);

  bool face_detected = face_detector_.detected() + 1; // +1 because detector returns -1 for no face and 0 for face detected!
  if(face_detected) {

    publishDebugImage(frame);
    
    publishFacePosition(frame);
  }
  else {
    RCLCPP_INFO(this->get_logger(), "No face detected");
  }  
}

cv::Mat CameraHLD::convertImageMsgToCvMat(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  return cv_bridge::toCvShare(image_msg, "bgr8")->image;
}

void CameraHLD::publishDebugImage(const cv::Mat& frame)
{
    cv::Rect face_roi = face_detector_.get_face_roi();

    cv::Mat cam_frame, iris_roi_frame;
    cam_frame = frame.clone();

    /* Draw the face roi rectangle on the captured camera frame */
    cv::rectangle(cam_frame, face_roi, cv::Scalar(0, 255, 0), 2); // Green rectangle will be drawn around detected face

    /* Get the face landmarks for eye-roi calculation */
    std::array<cv::Point, CLFML::FaceDetection::NUM_OF_FACE_DETECTOR_LANDMARKS> face_keypoints = face_detector_.get_face_landmarks();

    /* Draw the face landmarks on top of the captured camera frame */
    for (cv::Point keypoint : face_keypoints)
    {
        cv::circle(cam_frame, keypoint, 2, cv::Scalar(0, 255, 0), -1);
    }

    float distance_to_face = (FACE_WIDTH_CM * FOCAL_LENGTH_PIXEL) / face_roi.width; 

    // put distance to face on the image
    std::string distance_to_face_str = "Distance to face: " + std::to_string(distance_to_face) + " cm";
    cv::putText(cam_frame, distance_to_face_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    auto debug_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cam_frame).toImageMsg();
    debug_image_pub_->publish(*debug_image_msg);
}

void CameraHLD::publishFacePosition(const cv::Mat& frame) 
{
  std::array<cv::Point, CLFML::FaceDetection::NUM_OF_FACE_DETECTOR_LANDMARKS> face_keypoints = face_detector_.get_face_landmarks();
  cv::Rect face_roi = face_detector_.get_face_roi(); // get the face region of interest (a rectangle)
  cv::Point center_of_face = getCenterOfFace(face_roi);

  const double pixels_per_mm_reference = 2.0889;
  const double object_reference_width_mm = 200.0;
  const double distance_to_face_mm = getDistanceToFace(face_roi);
  const double adjusted_pixels_per_mm = pixels_per_mm_reference * (object_reference_width_mm / distance_to_face_mm);
  const double face_center_x_mm = (center_of_face.x - (640 / 2.0)) / adjusted_pixels_per_mm; // 640 is imagebreedte
  const double face_center_y_mm = (center_of_face.y - (480 / 2.0)) / adjusted_pixels_per_mm; // 480 is imagehoogte

  center_of_face.x = face_center_x_mm;
  center_of_face.y = face_center_y_mm;

  auto face_position_msg = createFacePositionMsg(center_of_face, distance_to_face_mm);

  face_position_pub_->publish(face_position_msg);
}

cv::Point CameraHLD::getCenterOfFace(const cv::Rect& face_roi) 
{
  return cv::Point(face_roi.x + face_roi.width/2, face_roi.y + face_roi.height/2);
}

float CameraHLD::getDistanceToFace(const cv::Rect &face_roi) 
{
  float distance = (FOCAL_LENGTH_PIXEL * (FACE_WIDTH_CM*10)) / face_roi.width;
  return distance;
}

geometry_msgs::msg::PointStamped CameraHLD::createFacePositionMsg(const cv::Point center_of_face, float distance_to_face)
{
  geometry_msgs::msg::PointStamped face_position_msg = geometry_msgs::msg::PointStamped();
  face_position_msg.header.stamp = this->get_clock()->now();
  face_position_msg.header.frame_id = tf_frame_id_;

  //Convert OpenCV coordinates to ROS Right-Hand Rule
  face_position_msg.point.x = distance_to_face;  // X = forward (distance to face) (positive)
  face_position_msg.point.y = -1 * center_of_face.x; // Y =  left (positive)
  face_position_msg.point.z = -1 * center_of_face.y; // Z = up (positive)

  //Convert to meters (Expected input: mm -> convert to meters)
  face_position_msg.point.x =  face_position_msg.point.x / 1000;
  face_position_msg.point.y =  face_position_msg.point.y / 1000; 
  face_position_msg.point.z =  face_position_msg.point.z / 1000;

  // DEBUG INFO
  double yaw = std::atan2(face_position_msg.point.y, face_position_msg.point.x) * 180.0 / M_PI;
  double pitch = std::atan2(face_position_msg.point.z, std::sqrt(std::pow(face_position_msg.point.x, 2) + std::pow(face_position_msg.point.y, 2))) * 180 / M_PI;
  RCLCPP_INFO(this->get_logger(), "Face detected at x: %f, y: %f, z: %f", face_position_msg.point.x, face_position_msg.point.y, face_position_msg.point.z);
  RCLCPP_INFO(this->get_logger(), "Yaw: %f, Pitch: %f", yaw, pitch);

  return face_position_msg;
}

}  // namespace camera_hld