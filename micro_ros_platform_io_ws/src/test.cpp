#include <Arduino.h>
#include <string>

// WifiConfig: Oorspronkelijke volgorde
struct WifiConfigOriginal
{
    std::string ssid_;
    std::string password_;
    std::string ip_;
    uint16_t port_;
};

// WifiConfig: Geoptimaliseerde volgorde
struct WifiConfigOptimized
{
    uint16_t port_;
    std::string ssid_;
    std::string password_;
    std::string ip_;
};

// UartConfig: Oorspronkelijke volgorde
struct UartConfigOriginal
{
    uint32_t baudrate_; // 4 bytes
    uint8_t uart_num_;  // 1 byte
    uint8_t rx_pin_;    // 1 byte
    uint8_t tx_pin_;    // 1 byte
};

// UartConfig: Geoptimaliseerde volgorde
struct UartConfigOptimized
{
    uint8_t uart_num_;  // 1 byte
    uint8_t rx_pin_;    // 1 byte
    uint8_t tx_pin_;    // 1 byte
    uint32_t baudrate_; // 4 bytes
};

void setup() {
    Serial.begin(115200);
    delay(1000); // Wacht op de seriële verbinding

    // WifiConfig Tests
    Serial.println("===== WifiConfig Tests =====");
    Serial.print("Size of WifiConfigOriginal: ");
    Serial.println(sizeof(WifiConfigOriginal));
    Serial.print("Size of WifiConfigOptimized: ");
    Serial.println(sizeof(WifiConfigOptimized));

    Serial.println("\nOffsets in WifiConfigOriginal:");
    Serial.print("Offset of ssid_: ");
    Serial.println((size_t)offsetof(WifiConfigOriginal, ssid_));
    Serial.print("Offset of password_: ");
    Serial.println((size_t)offsetof(WifiConfigOriginal, password_));
    Serial.print("Offset of ip_: ");
    Serial.println((size_t)offsetof(WifiConfigOriginal, ip_));
    Serial.print("Offset of port_: ");
    Serial.println((size_t)offsetof(WifiConfigOriginal, port_));

    Serial.println("\nOffsets in WifiConfigOptimized:");
    Serial.print("Offset of port_: ");
    Serial.println((size_t)offsetof(WifiConfigOptimized, port_));
    Serial.print("Offset of ssid_: ");
    Serial.println((size_t)offsetof(WifiConfigOptimized, ssid_));
    Serial.print("Offset of password_: ");
    Serial.println((size_t)offsetof(WifiConfigOptimized, password_));
    Serial.print("Offset of ip_: ");
    Serial.println((size_t)offsetof(WifiConfigOptimized, ip_));

    // UartConfig Tests
    Serial.println("\n===== UartConfig Tests =====");
    Serial.print("Size of UartConfigOriginal: ");
    Serial.println(sizeof(UartConfigOriginal));
    Serial.print("Size of UartConfigOptimized: ");
    Serial.println(sizeof(UartConfigOptimized));

    Serial.println("\nOffsets in UartConfigOriginal:");
    Serial.print("Offset of baudrate_: ");
    Serial.println((size_t)offsetof(UartConfigOriginal, baudrate_));
    Serial.print("Offset of uart_num_: ");
    Serial.println((size_t)offsetof(UartConfigOriginal, uart_num_));
    Serial.print("Offset of rx_pin_: ");
    Serial.println((size_t)offsetof(UartConfigOriginal, rx_pin_));
    Serial.print("Offset of tx_pin_: ");
    Serial.println((size_t)offsetof(UartConfigOriginal, tx_pin_));

    Serial.println("\nOffsets in UartConfigOptimized:");
    Serial.print("Offset of uart_num_: ");
    Serial.println((size_t)offsetof(UartConfigOptimized, uart_num_));
    Serial.print("Offset of rx_pin_: ");
    Serial.println((size_t)offsetof(UartConfigOptimized, rx_pin_));
    Serial.print("Offset of tx_pin_: ");
    Serial.println((size_t)offsetof(UartConfigOptimized, tx_pin_));
    Serial.print("Offset of baudrate_: ");
    Serial.println((size_t)offsetof(UartConfigOptimized, baudrate_));
}

void loop() {
    // Niets te doen in de loop
}
