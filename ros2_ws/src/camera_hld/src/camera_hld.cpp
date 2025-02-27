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

//experimental
constexpr double IRIS_DIAMETER_MM = 11.7;
constexpr double IRIS_DIAMETER_CM = IRIS_DIAMETER_MM / 10.0;


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
  iris_mesh_.load_model(CLFML_IRIS_MESH_CPU_MODEL_PATH);

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
    // Publish the debug image (contains the face ROI and iris keypoints and eye ROI)
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

    /* Calculate the Eye-regions of interest on the face using the facial keypoints */
    std::array<cv::Rect, 2> eye_rois = calculateEyeRoi(face_keypoints[0], face_keypoints[1]);

    uint8_t eye_index = 0; // for this demo 0 is left eye and 1 is right eye

    float biggest_iris_diameter = 0.0f;

    /* Do inference for both Eye's and draw the iris keypoints on the camera frame */
    for (cv::Rect &eye_roi : eye_rois)
    {
        /* Check if the eye_roi is within the bounds of the image */
        if (is_roi_within_bounds(eye_roi, cam_frame))
        {
            /* Draw the eye_roi on the camera frame */
            cv::rectangle(cam_frame, eye_roi, cv::Scalar(255, 0, 0), 2);
            /* Crop the eye_roi region from the camera frame */
            iris_roi_frame = cam_frame(eye_roi);

            // Check if the cropped iris ROI is empty (sanity check)
            if (iris_roi_frame.empty()) {
              RCLCPP_ERROR(this->get_logger(), "Cropped iris ROI is empty.");
              continue;
            }

            /* Do inference! */
            iris_mesh_.load_image(iris_roi_frame, eye_roi);
            /* Get the iris mesh keypoints from the model inference output */
            std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_keypoints = iris_mesh_.get_iris_mesh_points();
            /* Draw the iris keypoints on the camera frame (as 2D points) */
            for (cv::Point3f keypoint : iris_mesh_keypoints)
            {
              cv::circle(cam_frame, cv::Point(keypoint.x, keypoint.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            float iris_diameter = getIrisDiameterInPixel(iris_mesh_keypoints);
            //iris_diameter = iris_diameter / 640.0f; // Normalize the iris diameter to the image width
            biggest_iris_diameter = std::max(biggest_iris_diameter, iris_diameter);
        }
        else
        {
            std::string eye_label = (eye_index == 0) ? "Left" : "Right";
            std::cerr << "Warning: " << eye_label <<" eye ROI is out of image bounds and will be skipped." << std::endl;
        }
        ++eye_index;
    }

    //RCLCPP_INFO(this->get_logger(), "Biggest iris diameter: %f", biggest_iris_diameter);

    if(biggest_iris_diameter == 0.0f) {
      RCLCPP_INFO(this->get_logger(), "No iris detected, distance to face cannot be calculated.");
      return;
    }

    /*Option 1: Current implementation based on face width*/
    float distance_to_face = (FACE_WIDTH_CM * FOCAL_LENGTH_PIXEL) / face_roi.width; 
    // float distance_to_face = (29.7f * FOCAL_LENGTH_PIXEL) / face_roi.width; 
    //------------------------------------------------------------------------------------------------
    /*Option 2: Expirmantal with iris */
    // const double focal_length_mm = 2.7;
    // const double iris_diameter_mm = 11.7;
    // const double iris_diameter_pixel = biggest_iris_diameter;
    // const double pixel_size_mm = 0.0014;

    // double distance_to_face = (iris_diameter_mm * focal_length_mm) / (iris_diameter_pixel*pixel_size_mm);
    // distance_to_face = distance_to_face / 10.0f; //convert to CM
    //------------------------------------------------------------------------------------------------

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

  /*option 1: calculate distance to face with iris (for now not good!, because it seems i get pupil detection instead of iris!!!)*/
  // auto eye_roi =  calculateEyeRoi(face_keypoints[0], face_keypoints[1]); 
  // float irisSizePixel = getBiggestIrisDiameterInPixel(eye_roi, frame);
  // float distance_to_face_mm =  (IRIS_DIAMETER_MM * FOCAL_LENGTH_PIXEL) / irisSizePixel;
  // distance_to_face = distance_to_face / 10.0f; //convert to CM
  //----------------------------------------------------------------------------------------------------------

  //Current implementation based on face width
  const double pixels_per_mm_reference = 2.0889;
  const double object_reference_width_mm = 200.0;
  const double distance_to_face_mm = getDistanceToFace(face_roi);
  const double adjusted_pixels_per_mm = pixels_per_mm_reference * (object_reference_width_mm / distance_to_face_mm);
  const double face_center_x_mm = (center_of_face.x - (640 / 2.0)) / adjusted_pixels_per_mm; // 640 is imagebreedte
  const double face_center_y_mm = (center_of_face.y - (480 / 2.0)) / adjusted_pixels_per_mm; // 480 is imagehoogte

  center_of_face.x = face_center_x_mm;
  center_of_face.y = face_center_y_mm;
  //------------------------------------------------------------------------------------------------

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

  //Convert to CM (Expected input: mm -> convert to cm)
  //For testing purpuses everthing is in CM, later x and y should be converted to meters and z to cm
  face_position_msg.point.x =  face_position_msg.point.x / 10;
  face_position_msg.point.y =  face_position_msg.point.y / 10; 
  face_position_msg.point.z =  face_position_msg.point.z / 10;

  // DEBUG INFO
  double yaw = std::atan2(face_position_msg.point.y, face_position_msg.point.x) * 180.0 / M_PI;
  double pitch = std::atan2(face_position_msg.point.z, std::sqrt(std::pow(face_position_msg.point.x, 2) + std::pow(face_position_msg.point.y, 2))) * 180 / M_PI;
  RCLCPP_INFO(this->get_logger(), "Face detected at x: %f, y: %f, z: %f", face_position_msg.point.x, face_position_msg.point.y, face_position_msg.point.z);
  RCLCPP_INFO(this->get_logger(), "Yaw: %f, Pitch: %f", yaw, pitch);

  return face_position_msg;
}


const std::array<cv::Rect, 2> CameraHLD::calculateEyeRoi(const cv::Point &left_eye_landmark, const cv::Point &right_eye_landmark)
{
    std::array<cv::Rect, 2> ret;
    int roi_size = ((right_eye_landmark.x - left_eye_landmark.x) / 2);
    ret.at(0) = cv::Rect(left_eye_landmark.x - (roi_size / 2), left_eye_landmark.y - (roi_size / 2), roi_size, roi_size);
    ret.at(1) = cv::Rect(right_eye_landmark.x - (roi_size / 2), right_eye_landmark.y - (roi_size / 2), roi_size, roi_size);
    return ret;
}

float CameraHLD::getBiggestIrisDiameterInPixel(const std::array<cv::Rect, 2> &eye_rois, const cv::Mat& frame) 
{
  cv::Mat iris_roi_frame;
  float biggest_iris_diameter = 0.0f;

  for (const cv::Rect &eye_roi : eye_rois) {
    if(is_roi_within_bounds(eye_roi, frame)) { 
      /* Crop the eye_roi region from the camera frame */
      iris_roi_frame = frame(eye_roi);
      
      if (iris_roi_frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Cropped iris ROI is empty.");
        continue;
      }

      /* Do inference! */
      iris_mesh_.load_image(iris_roi_frame, eye_roi);

      /* Get the iris mesh keypoints from the model inference output */
      std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_keypoints = iris_mesh_.get_iris_mesh_points();
      
      float iris_diameter = getIrisDiameterInPixel(iris_mesh_keypoints);
      biggest_iris_diameter = std::max(biggest_iris_diameter, iris_diameter);
    }
  }

  return biggest_iris_diameter;
}

float CameraHLD::getIrisDiameterInPixel(const std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_landmarks)
{
    /** API returns 5x3D Iris landmarks;
     * Index 0: Iris Center
     * Index 1: Pupil Right
     * Index 2: Pupil Top
     * Index 3: Pupil Left
     * Index 4: Pupil Bottom
     */

    // Calculate the distance between the Pupil Right and Pupil Left
    float distance = cv::norm(iris_mesh_landmarks[1] - iris_mesh_landmarks[3]);    
    return distance;
}

bool CameraHLD::is_roi_within_bounds(const cv::Rect &roi, const cv::Mat &image)
{
  return (roi.x >= 0 && roi.y >= 0 && 
          roi.x + roi.width <= image.cols && 
          roi.y + roi.height <= image.rows);
}

}  // namespace camera_hld