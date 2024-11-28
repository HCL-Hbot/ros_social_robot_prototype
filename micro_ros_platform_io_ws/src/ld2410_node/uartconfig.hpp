#ifndef UART_CONFIG_HPP
#define UART_CONFIG_HPP

#include <stdint.h>

struct UartConfig
{
    uint8_t rx_pin_;    // RX pin
    uint8_t tx_pin_;    // TX pin
    uint8_t uart_num_;  // UART nummer
    uint32_t baudrate_; // Baudrate
};

#endif // UART_CONFIG_HPP
