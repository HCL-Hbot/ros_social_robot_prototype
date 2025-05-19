# Camera calibration

This directory contains python scripts to calibrate the mounted camara of MIKA.
Altough a one time calibration is enough i.e. the current calibration/settings is sufficient. These script can help to retreive new calibration value if the current setting is not good enough or if we switch to a new camera.

The following variable values of [camera_hld.cpp](../../ros2_ws/src/camera_hld/src/camera_hld.cpp) (camera package) can be retreived with the python scripts.

| camera_hld.cpp variable name| line number | needed python script|
|-----------------------------|-------------|---------------------|
|``constexpr double FOCAL_LENGTH_PIXEL``| 14 | ``calibration_a4v1.py`` or ``calibration_a4v2.py``|
|``const double pixels_per_mm_reference``| 107 | ``pixel_to_unit.py``|
|``const double object_reference_width_mm``| 108 | This should be the distance in mm where you have placed to object for the pixel_to_unit.py. In my case this was a box placed 200mm from the camera (see my_photo-13.jpg) |


 ## Installation 

 ```bash
python3 -m venv venv #Create a virtuel environment named venv
source venv/bin/activate #Activate the virutal environment
pip install -r requirements.txt
 ```

## Usage of python script

To retreive the focal length of the camera in pixels use one of the scripts below.
 ```bash
python3 ./calibration_a4v1.py
python3 ./calibration_a4v2.py
 ```
The ROI should be selected manually be creating a square with your mouse. Hold left mouse click and drag it over the desired object until it is fully coverd with a blue square. If this size of the square is ''good enough'' then press enter. The output will be shown in the terminal and written to a json file.

The retreive the pixel per mm reference :
 ```bash
python3 ./pixel_to_unit.py
 ```
 The ROI should be selected by clicking the corners of the object. A square will be drawn based on this. If the size of the square is ''good enough'' then press enter. The output will be shown in the terminal.


Some example output of the scripts
```bash
#calibration_a4v1.py
(venv) MBP-van-Agit:camera_calibration agit$ python3 ./calibration_a4v1.py 
Select a ROI and then press SPACE or ENTER button!
Cancel the selection process by pressing c button!
#output
Berekende focal length: 442.09 pixels
Focal length opgeslagen in focal_length.json
```

```bash
#calibration_a4v2.py
(venv) MBP-van-Agit:camera_calibration agit$ python3 ./calibration_a4v2.py 
Select a ROI and then press SPACE or ENTER button!
Cancel the selection process by pressing c button!
#output
Berekende focal length: 422.22 pixels
Focal length opgeslagen in focal_length.json
```

```bash
#pixel_to_unit.py
(venv) MBP-van-Agit:camera_calibration agit$ python3 ./
pixel_to_unit.py 
#output
Breedte in pixels: 279.00
Pixels per mm: 2.0667
```


# Note
The used images for calibration are 640x480, because the camera_hld currently is only supporting 640x480 images.