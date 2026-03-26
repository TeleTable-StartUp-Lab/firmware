#pragma once

#include <Arduino.h>
#include <MFRC522.h>
#include <SPI.h>

class RfidRc522Sensor
{
public:
    struct Config
    {
        SPIClass *spi;
        gpio_num_t sck_pin;
        gpio_num_t miso_pin;
        gpio_num_t mosi_pin;
        gpio_num_t ss_pin;
        gpio_num_t rst_pin;
        uint32_t poll_period_ms;
    };

    struct Reading
    {
        String uid_hex;
        String card_type;
        uint8_t sak = 0;
        uint8_t uid_size = 0;
    };

    explicit RfidRc522Sensor(const Config &cfg);

    bool begin();
    bool update(uint32_t now_ms);

    bool isOk() const;
    bool hasReading() const;
    uint8_t version() const;
    const Reading &reading() const;

private:
    Config cfg_;
    MFRC522 reader_;
    bool ok_ = false;
    bool has_reading_ = false;
    uint8_t version_ = 0;
    uint32_t last_poll_ms_ = 0;
    Reading reading_;

    bool readCard_();
};
