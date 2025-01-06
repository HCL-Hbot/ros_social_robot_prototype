#This custom ROS message is inteded for retreiving target data "frame" in "normal working mode". (NOT engineer mode)
#Target data frame is part of intra-frame which is a part of "Reporting data frame format" See page 16 and 17 of serial protocol LD2410B

uint8 rader_id #From which rader did we receive this

# Constants for target_state
uint8 NO_TARGET = 0
uint8 MOVING_ONLY = 1
uint8 STATIONARY_ONLY = 2
uint8 MOVING_AND_STATIONARY = 3

uint8 target_state #See constants for possible states
uint16 movement_distance #From the sensor to the target, measurment in cm.
uint8 movement_energy #Range [0-100]. Strength of the radar signal that has been reflected back from a target.
uint16 stationaty_distance #From the sensor to the target, measurment in cm.
uint8 stationaty_energy #Range [0-100]. Strength of the radar signal that has been reflected back from a target.
uint16 detection_distance #Measurement in cm. From the sensor to the target? (No explanation in the serial protocol and datasheet)