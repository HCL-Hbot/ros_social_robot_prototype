#ifndef WIFI_CONFIG_HPP
#define WIFI_CONFIG_HPP

#include <stdint.h>
#include <string>

struct WifiConfig
{
    uint16_t port_;
    std::string ip_;
    std::string ssid_;
    std::string password_;
};

#endif // WIFI_CONFIG_HPP
