#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_hld.hpp"
constexpr const char* DEFAULT_NODE_NAME = "camera_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB = "raw_image";
constexpr const char* DEFAULT_TOPIC_NAME_PUB = "face_info";
constexpr const char* TF_CAMERA_FRAME_ID_PARAMETER = "tf_frame_id";
constexpr const char* DEFAULT_TF_CAMERA_FRAME_ID = "camera";

constexpr float IRIS_DIAMETER_MM = 11.7f;
constexpr float IRIS_DIAMETER_CM = IRIS_DIAMETER_MM / 10.0f;

CameraHLD::CameraHLD() : 
  rclcpp::Node(DEFAULT_NODE_NAME),
  raw_image_sub_(create_subscription<sensor_msgs::msg::Image>(
            DEFAULT_TOPIC_NAME_SUB, 10, std::bind(&CameraHLD::imageCallback, this, std::placeholders::_1)))
{
  tf_frame_id_ = this->declare_parameter<std::string>(TF_CAMERA_FRAME_ID_PARAMETER, DEFAULT_TF_CAMERA_FRAME_ID);
  if(tf_frame_id_== DEFAULT_TF_CAMERA_FRAME_ID)
  {
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
  
  if(frame.empty()) return; // no need to to processing on empty image.

  face_detector_.load_image(frame);

  bool face_detected = face_detector_.detected() + 1; // +1 because detector returns -1 for no face and 0 for face detected!
  if(face_detected)
  {
    //RCLCPP_INFO(this->get_logger(), "Image width @ %d, height @ %d", frame.size().width, frame.size().height);
    publishFacePosition(frame);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No face detected");
  }  
}

cv::Mat CameraHLD::convertImageMsgToCvMat(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  return cv_bridge::toCvShare(image_msg, "bgr8")->image;
}

void CameraHLD::publishFacePosition(const cv::Mat& frame)
{
   std::array<cv::Point, CLFML::FaceDetection::NUM_OF_FACE_DETECTOR_LANDMARKS> face_keypoints = face_detector_.get_face_landmarks();
   cv::Rect face_roi = face_detector_.get_face_roi(); // get the face region of interest (a rectangle)
   cv::Point center_of_face = getCenterOfFace(face_roi);

   cv::Rect eye_roi = getEyeRoi(face_keypoints[0], face_keypoints[1]); //mockup for now, needed for distance calculation
   float distance_to_face = getDistanceToFace(face_roi, eye_roi, frame.size().width); //mockup for now, needed for distance calculation
   auto face_position_msg = createFacePositionMsg(center_of_face, distance_to_face);

  face_position_pub_->publish(face_position_msg);
}

cv::Point CameraHLD::getCenterOfFace(const cv::Rect& face_roi)
{
  return cv::Point(face_roi.x + face_roi.width/2, face_roi.y + face_roi.height/2);
}

cv::Rect CameraHLD::getEyeRoi(const cv::Point& left_eye_landmark, const cv::Point& right_eye_landmark)
{
  return cv::Rect();
}

float CameraHLD::getDistanceToFace(const cv::Rect &face_roi, const cv::Rect &eye_roi, uint32_t image_width)
{
  return 80.0f; //CM
}

geometry_msgs::msg::PointStamped CameraHLD::createFacePositionMsg(const cv::Point center_of_face, float distance_to_face)
{
  geometry_msgs::msg::PointStamped face_position_msg = geometry_msgs::msg::PointStamped();
  face_position_msg.header.stamp = this->get_clock()->now();
  face_position_msg.header.frame_id = tf_frame_id_;

  // Convert OpenCV coordinates to ROS Right-Hand Rule
  face_position_msg.point.x = distance_to_face;  // X = forward (distance to camera)
  face_position_msg.point.y = center_of_face.x; // Y =  left/right
  face_position_msg.point.z = center_of_face.y; // Z = up/down

  return face_position_msg;
}

//--------------TODO---------------------
// FUNCTIONS HERE BELOW ARE NOT IMPLEMENTED OR USED YET. NEED TO REVIEW IF THESE ARE NEEDED OR NOT. 
// THE GOALS WAS TO USE THESE TO CALCULATE THE POSITION OF THE FACE IN THE CAMERA FRAME AND DEPTH OF FACE BY USING THE IRIS AS A REFERENCE POINT. 

// void CameraHLD::publishFacePosition(const cv::Mat &frame)
// {
//   /* Get the face landmarks for eye-roi calculation */
//   std::array<cv::Point, CLFML::FaceDetection::NUM_OF_FACE_DETECTOR_LANDMARKS> face_keypoints = face_detector_.get_face_landmarks();

//   cv::Point left_eye_landmark = face_keypoints[0];
//   cv::Point right_eye_landmark = face_keypoints[1];

//   /* Calculate the Eye-regions of interest on the face using the facial keypoints */
//   std::array<cv::Rect, 2> eye_rois = calculate_eye_roi(left_eye_landmark, right_eye_landmark);
  
//   /*option 1 */
//   // float iris_diameter_pixel = getBiggestIrisDiameterInPixel(eye_rois);

//   // float distance_to_iris = getDistanceFromIrisToCamera(iris_diameter_pixel);
//   //------------------------------------------------------------------------------------------------
  
//   /*option 2 */
//   // int frame_width = 680;
//   // cv::Rect biggest_roi = getBiggestIrisRoi(eye_rois);
//   // float distance_to_iris = calculateCameraDistance(biggest_roi, frame_width);
//   //------------------------------------------------------------------------------------------------

//   /*option 3 HARDCODED VALUE FOR NOW*/
//   float distance_to_iris = 80.0f; //CM 
//   //------------------------------------------------------------------------------------------------
//   cv::Rect face_roi = face_detector_.get_face_roi(); // get the face region of interest (a rectangle)
//   int center_face_x = face_roi.x + face_roi.width/2;
//   int center_face_y = face_roi.y + face_roi.height/2;
  
//   RCLCPP_INFO(this->get_logger(), "Face detected at x: %d, y: %d, z: %f, width: %d, height: %d", center_face_x, center_face_y, distance_to_iris, face_roi.width, face_roi.height);
  
//   geometry_msgs::msg::PointStamped face_position_msg = geometry_msgs::msg::PointStamped();
//   face_position_msg.header.stamp = this->get_clock()->now();
//   face_position_msg.header.frame_id = tf_frame_id_;

//   // Convert OpenCV coordinates to ROS Right-Hand Rule
//   face_position_msg.point.x = distance_to_iris;  // X = forward (distance to camera)
//   face_position_msg.point.y = center_face_x; // Y =  left/right
//   face_position_msg.point.z = center_face_y; // Z = up/down

//   face_position_pub_->publish(face_position_msg);
// }

const std::array<cv::Rect, 2> CameraHLD::calculate_eye_roi(const cv::Point &left_eye_landmark, const cv::Point &right_eye_landmark)
{
    std::array<cv::Rect, 2> ret;
    int roi_size = ((right_eye_landmark.x - left_eye_landmark.x) / 2);
    ret.at(0) = cv::Rect(left_eye_landmark.x - (roi_size / 2), left_eye_landmark.y - (roi_size / 2), roi_size, roi_size);
    ret.at(1) = cv::Rect(right_eye_landmark.x - (roi_size / 2), right_eye_landmark.y - (roi_size / 2), roi_size, roi_size);
    return ret;
}

float CameraHLD::getBiggestIrisDiameterInPixel(const std::array<cv::Rect, 2> &eye_rois)
{
  cv::Mat eye_roi_frame; 
  cv::Mat iris_roi_frame;
  float biggest_iris_diameter = 0.0f;

  for (const cv::Rect &eye_roi : eye_rois)
  {
    if(is_roi_within_bounds(eye_roi, eye_roi_frame)) //is this correct?
    {
      /* Crop the eye_roi region from the camera frame */
      iris_roi_frame = eye_roi_frame(eye_roi);
      
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
    double distance = cv::norm(iris_mesh_landmarks[1] - iris_mesh_landmarks[3]);    
    return static_cast<float>(distance);
}

bool CameraHLD::is_roi_within_bounds(const cv::Rect &roi, const cv::Mat &image)
{
  return (roi.x >= 0 && roi.y >= 0 && 
          roi.x + roi.width <= image.cols && 
          roi.y + roi.height <= image.rows);
}

float CameraHLD::getDistanceFromIrisToCamera(float iris_diameter_in_pixel)
{
     // Focal length of the camera in pixels (this needs to be calibrated)
    const float focal_length_px = 500.0f; // Example value, you need to calibrate this

    // Calculate the distance from the camera to the iris
    float distance = (IRIS_DIAMETER_CM * focal_length_px) / iris_diameter_in_pixel;
    
    return distance;
}

float CameraHLD::calculateCameraDistance(const cv::Rect &eye_roi, uint32_t image_width)
{
    // Average size of human iris in mm
    const float known_iris_diameter_mm = 11.7f;
    cv::Mat eye_roi_frame; 
    cv::Mat iris_roi_frame;

    /* Crop the eye_roi region from the camera frame */
    iris_roi_frame = eye_roi_frame(eye_roi);

    iris_mesh_.load_image(iris_roi_frame, eye_roi);

    // Get the iris mesh landmarks for the given eye ROI
    std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_landmarks = iris_mesh_.get_iris_mesh_points();

    // Iris points are [center, left, top, right, bottom]
    const cv::Point3f &left = iris_mesh_landmarks[1];
    const cv::Point3f &right = iris_mesh_landmarks[3];

    // Calculate iris size in pixels (normalized to 0..1)
    const float irisSize = std::abs(left.x - right.x) / static_cast<float>(image_width);

    // Calculate camera distance in meters
    const float cameraDistance = (0.5f * known_iris_diameter_mm / irisSize) / 1000.0f;

    return cameraDistance;
}


cv::Rect CameraHLD::getBiggestIrisRoi(const std::array<cv::Rect, 2> &eye_rois)
{
    cv::Mat eye_roi_frame; 
    cv::Mat iris_roi_frame;
    cv::Rect biggest_iris_roi;
    float max_iris_diameter = 0.0f;

    for (const cv::Rect &eye_roi : eye_rois)
    {
        if (is_roi_within_bounds(eye_roi, eye_roi_frame)) //is this correct?
        {
          /* Crop the eye_roi region from the camera frame */
          iris_roi_frame = eye_roi_frame(eye_roi);
          
          /* Do inference! */
          iris_mesh_.load_image(iris_roi_frame, eye_roi);

          /* Get the iris mesh keypoints from the model inference output */
          std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_keypoints = iris_mesh_.get_iris_mesh_points();

          float iris_diameter = getIrisDiameterInPixel(iris_mesh_keypoints);
          if (iris_diameter > max_iris_diameter)
          {
              max_iris_diameter = iris_diameter;
              biggest_iris_roi = eye_roi;
          }
        }
    }

    return biggest_iris_roi;
}

// cv::Rect calculateEyeRoi(cv::Point leftMoft, cv::Point rightMost) const{
//     int cx = (leftMoft.x + rightMost.x) / 2;
//     int cy = (leftMoft.y + rightMost.y) / 2; 

//     int w = std::abs(leftMoft.x - rightMost.x);
//     int h = std::abs(leftMoft.y - rightMost.y);
//     w = h = std::max(w, h);
    
//     return cv::Rect(cx - w/2, cy - h/2, w, h);
// }

//--------------END TODO---------------------