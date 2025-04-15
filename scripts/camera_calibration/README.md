# Camera calibration

This directory contains python scripts to calibrate the mounted camara of MIKA.
Altough a one time calibration is enough i.e. the current calibration/settings is sufficient. These script can help to retreive new calibration value for recalibration if the current setting is not good enough or if we switch to a new camera.

The following variables of camera_hld.cpp (camera package) can be configured with the python scripts.
``constexpr double FOCAL_LENGTH_PIXEL``
``const double pixels_per_mm_reference``
``const double object_reference_width_mm``


TODO add python scripts and guide!!!