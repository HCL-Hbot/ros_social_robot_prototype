
#ifndef LD2410_RADAR_HPP_
#define LD2410_RADAR_HPP_

#include <Arduino.h>
#include "ld2410.h"

/*For this project only support for normal mode (Engineer mode not needed)*/
struct TargetFrameData
{
    uint8_t target_state_; 
    uint16_t movement_distance_; 
    uint8_t movement_energy_;
    uint16_t stationaty_distance_; 
    uint8_t stationaty_energy_;
    uint16_t detection_distance_;
};

/**
 * @brief Wrapper class for ld2410
 */
class Ld2410Radar
{
    public:
        Ld2410Radar();

        ~Ld2410Radar();

        bool begin(Stream& uart_radar);

        bool read();

        const TargetFrameData& getCurrentTargetFrame();

    private:
        ld2410 radar_sensor_;
        TargetFrameData target_frame_;
};
#endif //LD2410_RADAR_HPP_