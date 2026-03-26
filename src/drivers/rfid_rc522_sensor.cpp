#include "drivers/rfid_rc522_sensor.h"

RfidRc522Sensor::RfidRc522Sensor(const Config &cfg)
    : cfg_(cfg),
      reader_(static_cast<uint8_t>(cfg.ss_pin), static_cast<uint8_t>(cfg.rst_pin))
{
}

bool RfidRc522Sensor::begin()
{
    if (!cfg_.spi)
        return false;

    if (cfg_.rst_pin != GPIO_NUM_NC)
    {
        pinMode(static_cast<uint8_t>(cfg_.rst_pin), OUTPUT);
        digitalWrite(static_cast<uint8_t>(cfg_.rst_pin), LOW);
        delay(2);
        digitalWrite(static_cast<uint8_t>(cfg_.rst_pin), HIGH);
        delay(50);
    }

    cfg_.spi->begin(static_cast<int>(cfg_.sck_pin),
                    static_cast<int>(cfg_.miso_pin),
                    static_cast<int>(cfg_.mosi_pin),
                    static_cast<int>(cfg_.ss_pin));

    pinMode(static_cast<uint8_t>(cfg_.ss_pin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(cfg_.ss_pin), HIGH);

    reader_.PCD_Init(static_cast<uint8_t>(cfg_.ss_pin), static_cast<uint8_t>(cfg_.rst_pin));
    reader_.PCD_SetAntennaGain(MFRC522::RxGain_max);
    delay(4);

    version_ = reader_.PCD_ReadRegister(MFRC522::VersionReg);
    ok_ = (version_ != 0x00) && (version_ != 0xFF);
    has_reading_ = false;
    last_poll_ms_ = millis();
    return ok_;
}

bool RfidRc522Sensor::update(uint32_t now_ms)
{
    if (!ok_)
        return false;

    if (now_ms - last_poll_ms_ < cfg_.poll_period_ms)
        return false;

    last_poll_ms_ = now_ms;
    return readCard_();
}

bool RfidRc522Sensor::isOk() const
{
    return ok_;
}

bool RfidRc522Sensor::hasReading() const
{
    return has_reading_;
}

uint8_t RfidRc522Sensor::version() const
{
    return version_;
}

const RfidRc522Sensor::Reading &RfidRc522Sensor::reading() const
{
    return reading_;
}

bool RfidRc522Sensor::readCard_()
{
    if (!reader_.PICC_IsNewCardPresent())
        return false;

    if (!reader_.PICC_ReadCardSerial())
        return false;

    reading_.uid_hex = "";
    for (byte i = 0; i < reader_.uid.size; ++i)
    {
        char hex[4] = {0};
        snprintf(hex, sizeof(hex), "%02X", reader_.uid.uidByte[i]);
        if (i > 0)
            reading_.uid_hex += ':';
        reading_.uid_hex += hex;
    }

    reading_.sak = reader_.uid.sak;
    reading_.uid_size = reader_.uid.size;
    reading_.card_type = reader_.PICC_GetTypeName(reader_.PICC_GetType(reader_.uid.sak));

    has_reading_ = true;

    reader_.PICC_HaltA();
    reader_.PCD_StopCrypto1();
    return true;
}
