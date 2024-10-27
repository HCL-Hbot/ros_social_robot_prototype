# FaceInfo.msg
# Custom ROS message for detected face information

# Header for timestamp (also contains info about frame_id)
std_msgs/Header header

# Bounding box of the detected face (x, y, width, height)
int32 bounding_box_x
int32 bounding_box_y
int32 bounding_box_width
int32 bounding_box_height

# Orientation of the face relative to the camera as a quaternion
geometry_msgs/Quaternion orientation
