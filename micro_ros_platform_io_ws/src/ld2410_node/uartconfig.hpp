#ifndef UART_CONFIG_HPP_
#define UART_CONFIG_HPP_

#include <stdint.h>

/**
 * @struct UartConfig
 * @brief A structure to hold UART configuration details.
 *
 * This structure contains the necessary information to configure a UART connection,
 * including the RX pin, TX pin, UART number, and baud rate.
 */
struct UartConfig
{
    uint8_t rx_pin_;    /**< The RX pin for the UART connection. */
    uint8_t tx_pin_;    /**< The TX pin for the UART connection. */
    uint8_t uart_num_;  /**< The UART number. */
    uint32_t baudrate_; /**< The baud rate for the UART connection. */
};

#endif // UART_CONFIG_HPP_
