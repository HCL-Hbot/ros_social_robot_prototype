#EyeLidControl.msg

uint8 LEFT_EYE = 0
uint8 RIGHT_EYE = 1
uint8 BOTH_EYES = 2

uint8 eye_id # The id of the eye to control, should have one of the values above
float32 top_lid_position # The position of the upper lid, should be between 0 and 100.
float32 bottom_lid_position # The position of the lower lid, should be between 0 and 100.