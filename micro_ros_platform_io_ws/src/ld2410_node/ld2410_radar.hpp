#ifndef LD2410_RADAR_HPP_
#define LD2410_RADAR_HPP_

#include <Arduino.h>
#include "ld2410.h"

/*For this project only support for normal mode (Engineer mode not needed)*/

/**
 * @struct TargetFrameData
 * @brief A structure to hold target frame data from the LD2410 radar.
 *
 * This structure contains the necessary information about the target detected by the radar,
 * including the target state, movement distance, movement energy, stationary distance,
 * stationary energy, and detection distance.
 */
struct TargetFrameData
{
    uint8_t target_state_;        /**< The state of the target (e.g. NO_TARGET, MOVING_ONLY, STATIONARY_ONLY, MOVING_AND_STATIONARY). */
    uint16_t movement_distance_;  /**< The distance of the moving target. */
    uint8_t movement_energy_;     /**< The energy level of the moving target. */
    uint16_t stationaty_distance_;/**< The distance of the stationary target. */
    uint8_t stationaty_energy_;   /**< The energy level of the stationary target. */
    uint16_t detection_distance_; /**< Probably the overall detection distance. 
                                       This information is not well documented in the Manual nor the serial protocol.
                                       My guess is this variable will always have a value regardless of the target_state_,
                                       could be usefull when state is: MOVING_AND_STATIONARY or NO_TARGET  */
};

/**
 * @brief Wrapper class for ld2410
 *
 * This class provides a wrapper for the LD2410 library, allowing for initialization,
 * reading data, and retrieving the current target frame data. 
 * This class retreives the current target frame data specified as in the serial communcication protocol v1.4. of the ld2410B. 
 */
class Ld2410Radar
{
    public:
        /**
         * @brief Construct a new Ld2410Radar object.
         */
        Ld2410Radar();

        /**
         * @brief Destroy the Ld2410Radar object.
         */
        ~Ld2410Radar();

        /**
         * @brief Initialize the radar sensor.
         *
         * @param uart_radar The UART stream to communicate with the radar sensor.
         * @return true if initialization was successful, false otherwise.
         */
        bool begin(Stream& uart_radar);

        /**
         * @brief Read data from the radar sensor.
         *
         * @return true if data was successfully read, false otherwise.
         */
        bool read();

        /**
         * @brief Get the current target frame data.
         *
         * @return A reference to the current TargetFrameData structure.
         */
        const TargetFrameData& getCurrentTargetFrame();

    private:
        ld2410 radar_sensor_;      /**< The radar sensor object. */
        TargetFrameData target_frame_; /**< The current target frame data. */
};

#endif //LD2410_RADAR_HPP_