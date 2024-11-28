#include "ld2410_radar.hpp"


Ld2410Radar::Ld2410Radar():target_frame_({0,0,0,0,0,0})
{
}

Ld2410Radar::~Ld2410Radar()
{
}

bool Ld2410Radar::begin(Stream &uart_radar)
{
    return radar_sensor_.begin(uart_radar,true);
}

bool Ld2410Radar::read()
{
    return radar_sensor_.read();
}

const TargetFrameData &Ld2410Radar::getCurrentTargetFrame()
{
    const FrameData& fd = radar_sensor_.getFrameData();

    if(fd.data)
    {
        target_frame_.target_state_ = fd.data[8];
        target_frame_.movement_distance_ = fd.data[9] | (fd.data[10] << 8);
        target_frame_.movement_energy_ = fd.data[11];
        target_frame_.stationaty_distance_ = fd.data[12] | (fd.data[13] << 8);
        target_frame_.stationaty_energy_ = fd.data[14];
        target_frame_.detection_distance_ = fd.data[15] | (fd.data[16] << 8);
    }
    return target_frame_;
}
