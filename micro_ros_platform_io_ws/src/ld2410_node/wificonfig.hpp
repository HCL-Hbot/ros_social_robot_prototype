#ifndef WIFI_CONFIG_HPP_
#define WIFI_CONFIG_HPP_

#include <stdint.h>
#include <string>

namespace radar_lld {
/**
 * @struct WifiConfig
 * @brief A structure to hold WiFi configuration details.
 *
 * This structure contains the necessary information to configure a WiFi connection,
 * including the port number, IP address, SSID, and password.
 */
struct WifiConfig
{
    uint16_t port_;       /**< The port number for the WiFi connection. */
    std::string ip_;      /**< The IP address for the WiFi connection. */
    std::string ssid_;    /**< The SSID (network name) for the WiFi connection. */
    std::string password_;/**< The password for the WiFi connection. */
};

}

#endif // WIFI_CONFIG_HPP_
