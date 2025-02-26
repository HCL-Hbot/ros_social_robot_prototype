#include "camera_hld.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

constexpr const char* DEFAULT_NODE_NAME = "camera_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB = "raw_image";
constexpr const char* DEFAULT_TOPIC_NAME_PUB = "face_info";
constexpr const char* TF_CAMERA_FRAME_ID_PARAMETER = "tf_frame_id";
constexpr const char* DEFAULT_TF_CAMERA_FRAME_ID = "camera";
constexpr const char* DEBUG_TOPIC_NAME_PUB = "debug_image";

constexpr double FACE_WIDTH_CM = 16.0;
constexpr double IRIS_DIAMETER_MM = 11.7;
constexpr double IRIS_DIAMETER_CM = IRIS_DIAMETER_MM / 10.0;
constexpr double FOCAL_LENGTH_PIXEL = 487.50;

constexpr double KNOWN_PIXEL_WIDTH = 55.0; //pixel width of a known reference object
constexpr double KNOWN_WIDTH_CM = 100.0; 
constexpr double PIXEL_PER_CM = KNOWN_PIXEL_WIDTH / KNOWN_WIDTH_CM;

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
   // Draw the face ROI on the image
   //cv::Rect face_roi = face_detector_.get_face_roi();
   //cv::rectangle(frame, face_roi, cv::Scalar(0, 255, 0), 2);

    // Publish the debug image
    publishDebugImage(frame);
    
    //RCLCPP_INFO(this->get_logger(), "Image width @ %d, height @ %d", frame.size().width, frame.size().height);
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
    std::array<cv::Rect, 2> eye_rois = calculate_eye_roi(face_keypoints[0], face_keypoints[1]);

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

            if (iris_roi_frame.empty())
            {
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

    //float distance_to_face = (FACE_WIDTH_CM * FOCAL_LENGTH_PIXEL) / face_roi.width; 
    float distance_to_face = (29.7f * FOCAL_LENGTH_PIXEL) / face_roi.width; 

    // put distance to face on the image
    std::string distance_to_face_str = "Distance to face: " + std::to_string(distance_to_face) + " cm";
    cv::putText(cam_frame, distance_to_face_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    auto debug_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cam_frame).toImageMsg();
    debug_image_pub_->publish(*debug_image_msg);

    // cv::Point center_of_face = getCenterOfFace(face_roi);
    // float irisSizePixel = biggest_iris_diameter; //mockup for now, needed for distance calculation
    // RCLCPP_INFO(this->get_logger(), "Biggest iris diameter: %f", irisSizePixel);
    // float distance_to_face =  (IRIS_DIAMETER_MM * FOCAL_LENGTH_PIXEL) / irisSizePixel; //mockup for now, needed for distance calculation
    // distance_to_face = distance_to_face / 10.0f; //convert to CM
    // auto face_position_msg = createFacePositionMsg(center_of_face, distance_to_face);
    // face_position_pub_->publish(face_position_msg);

}

void CameraHLD::publishFacePosition(const cv::Mat& frame) 
{
  std::array<cv::Point, CLFML::FaceDetection::NUM_OF_FACE_DETECTOR_LANDMARKS> face_keypoints = face_detector_.get_face_landmarks();
  cv::Rect face_roi = face_detector_.get_face_roi(); // get the face region of interest (a rectangle)
  cv::Point center_of_face = getCenterOfFace(face_roi);

  /*option 1: calculate distance to face with iris (for now not good!, because it seems i get pupil detection instead of iris!!!)*/
  // auto eye_roi =  calculate_eye_roi(face_keypoints[0], face_keypoints[1]); 
  // float irisSizePixel = getBiggestIrisDiameterInPixel(eye_roi, frame);
  // float distance_to_face =  (IRIS_DIAMETER_MM * FOCAL_LENGTH_PIXEL) / irisSizePixel;
  // distance_to_face = distance_to_face / 10.0f; //convert to CM

  /*option 2: calculate distance to face with width of face*/
  //float distance_to_face = (FACE_WIDTH_CM * FOCAL_LENGTH_PIXEL) / face_roi.width;
  //float distance_to_face = getDistanceToFace(face_roi, eye_roi, frame.size().width); //mockup for now, needed for distance calculation

  const double reference_distance_mm = 1000.0f; // 1 meter
  const double known_width_of_a_object_mm = 297.0f; //pixel width of a known reference object
  const double distance_to_face_mm = (known_width_of_a_object_mm * FOCAL_LENGTH_PIXEL) / face_roi.width;
  const double pixels_per_mm = face_roi.width / known_width_of_a_object_mm;
  const double adjusted_pixels_per_mm = pixels_per_mm * (reference_distance_mm / distance_to_face_mm);
  const double face_center_x_mm = (center_of_face.x - (640 / 2.0)) / adjusted_pixels_per_mm; // 640 is een voorbeeld camerabreedte
  const double face_center_y_mm = (center_of_face.y - (480 / 2.0)) / adjusted_pixels_per_mm; // 480 is een voorbeeld camerahoogte

  center_of_face.x = face_center_x_mm;
  center_of_face.y = face_center_y_mm;

  auto face_position_msg = createFacePositionMsg(center_of_face, distance_to_face_mm);

  face_position_pub_->publish(face_position_msg);
}

cv::Point CameraHLD::getCenterOfFace(const cv::Rect& face_roi) 
{
  return cv::Point(face_roi.x + face_roi.width/2, face_roi.y + face_roi.height/2);
}

//Could be used to calculate the eye region of interest on the face (replacement for calculate_eye_roi)
cv::Rect CameraHLD::getEyeRoi(const cv::Point& left_eye_landmark, const cv::Point& right_eye_landmark)
{
  return cv::Rect();
}

//TODO - this function is not implemented yet.
float CameraHLD::getDistanceToFace(const cv::Rect &face_roi, const cv::Rect &eye_roi, uint32_t image_width) 
{
  //cv::Point center_of_face = getCenterOfFace(face_roi);
  //float distance = (16 * FOCAL_LENGTH_PIXEL) / face_roi.width; //mockup for now, needed for distance calculation
  //float irisSizePixel = getBiggestIrisDiameterInPixel(eye_roi);
  //float distance = (IRIS_DIAMETER_MM * FOCAL_LENGTH_PIXEL) / irisSizePixel; //mockup for now, needed for distance calculation
  //distance = distance / 10.0f; //convert to CM
  //float distance = (IRIS_DIAMETER_MM * FOCAL_LENGTH_PIXEL)/ eye_roi.width; //mockup for now, needed for distance calculation
  //distance = distance/10.0f; //convert to CM
  ///return distance;//80.0f; //CM
  return 80.0f;
}

geometry_msgs::msg::PointStamped CameraHLD::createFacePositionMsg(const cv::Point center_of_face, float distance_to_face)
{
  geometry_msgs::msg::PointStamped face_position_msg = geometry_msgs::msg::PointStamped();
  face_position_msg.header.stamp = this->get_clock()->now();
  face_position_msg.header.frame_id = tf_frame_id_;

  // constexpr float img_width = 640.0;
  // constexpr float img_height = 480.0;
  // constexpr double principal_x = img_width / 2.0;
  // constexpr double principal_y = img_height / 2.0;

  // float x_pixel = center_of_face.x;
  // float y_pixel = center_of_face.y;

  // // Zet centimeters om naar meters
  // double z_ros = distance_to_face / 100.0;
  // double x_ros = (x_pixel - principal_x) * z_ros / FOCAL_LENGTH_PIXEL;
  // double y_ros = -(y_pixel - principal_y) * z_ros / FOCAL_LENGTH_PIXEL;


  //Convert OpenCV coordinates to ROS Right-Hand Rule
  face_position_msg.point.x = distance_to_face;  // X = forward (distance to camera) (positive)
  face_position_msg.point.y = -1 * center_of_face.x; // Y =  left (positive)
  face_position_msg.point.z = -1 * center_of_face.y; // Z = up (positive)

  //cm
  face_position_msg.point.x =  face_position_msg.point.x / 10;
  face_position_msg.point.y =  face_position_msg.point.y / 10; 
  face_position_msg.point.z =  face_position_msg.point.z / 10;

  // double yaw = std::atan2(face_position_msg.point.y, face_position_msg.point.x) * 180.0 / M_PI;
  // //double pitch = std::atan2(face_position_msg.point.z, std::sqrt(std::pow(face_position_msg.point.x, 2) + std::pow(face_position_msg.point.y, 2))) * 180 / M_PI;
  // double pitch = std::atan2(face_position_msg.point.z, face_position_msg.point.x) * 180.0 / M_PI;
  // RCLCPP_INFO(this->get_logger(), "Face detected at x: %f, y: %f, z: %f", face_position_msg.point.x, face_position_msg.point.y, face_position_msg.point.z);
  // RCLCPP_INFO(this->get_logger(), "Yaw: %f, Pitch: %f", yaw, pitch);
  // face_position_msg.point.x = x_ros;
  // face_position_msg.point.y = y_ros;  
  // face_position_msg.point.z = z_ros;

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

float CameraHLD::getBiggestIrisDiameterInPixel(const std::array<cv::Rect, 2> &eye_rois, const cv::Mat& frame) 
{
  cv::Mat iris_roi_frame;
  float biggest_iris_diameter = 0.0f;
  //eye_roi, cam_frame
  for (const cv::Rect &eye_roi : eye_rois) {
    if(is_roi_within_bounds(eye_roi, frame)) { //is this correct?
      /* Crop the eye_roi region from the camera frame */
      iris_roi_frame = frame(eye_roi);
      
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

    for (const cv::Rect &eye_roi : eye_rois) {
        if (is_roi_within_bounds(eye_roi, eye_roi_frame)) { //is this correct?
        
          /* Crop the eye_roi region from the camera frame */
          iris_roi_frame = eye_roi_frame(eye_roi);
          
          /* Do inference! */
          iris_mesh_.load_image(iris_roi_frame, eye_roi);

          /* Get the iris mesh keypoints from the model inference output */
          std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_keypoints = iris_mesh_.get_iris_mesh_points();

          float iris_diameter = getIrisDiameterInPixel(iris_mesh_keypoints);
          if (iris_diameter > max_iris_diameter) {
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

}  // namespace camera_hld