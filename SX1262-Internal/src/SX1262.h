//ChatGPT modified SX1262 library
//August 7, 2026
//ESP32 Core 3.3.10
//Optimized for Ebyte-S3-900TB
//Untested other esp32-s3, sx1262 devices

#ifndef SX1262_HPP
#define SX1262_HPP

#include <Arduino.h>
#include <SPI.h>

class SX1262 {

private:

    uint8_t nssPin;
    uint8_t busyPin;
    uint8_t rstPin;
    uint8_t dio1Pin;


    void waitBusy()
    {
        while (digitalRead(busyPin) == HIGH) {
            delayMicroseconds(10);
        }
    }


    void spiWriteCmd(uint8_t opcode,
                     const uint8_t *params,
                     uint8_t length)
    {
        waitBusy();

        digitalWrite(nssPin, LOW);

        SPI.transfer(opcode);

        for(uint8_t i = 0; i < length; i++) {
            SPI.transfer(params[i]);
        }

        digitalWrite(nssPin, HIGH);

        waitBusy();
    }


    uint16_t spiReadCmd16(uint8_t opcode)
    {
        waitBusy();

        digitalWrite(nssPin, LOW);

        SPI.transfer(opcode);

        SPI.transfer(0x00);   // status byte

        uint8_t msb = SPI.transfer(0x00);
        uint8_t lsb = SPI.transfer(0x00);

        digitalWrite(nssPin, HIGH);

        waitBusy();

        return ((uint16_t)msb << 8) | lsb;
    }


public:


    SX1262(uint8_t cs,
           uint8_t busy,
           uint8_t rst,
           uint8_t dio1)
    :
    nssPin(cs),
    busyPin(busy),
    rstPin(rst),
    dio1Pin(dio1)
    {}


    // -------------------------------------------------
    // Hardware initialization
    // -------------------------------------------------

    void initHardware(bool resetRadio = true)
    {

        pinMode(nssPin, OUTPUT);
        digitalWrite(nssPin, HIGH);

        pinMode(busyPin, INPUT);
        pinMode(dio1Pin, INPUT);


        if(resetRadio)
        {
            pinMode(rstPin, OUTPUT);

            digitalWrite(rstPin, LOW);
            delay(5);

            digitalWrite(rstPin, HIGH);
            delay(10);

            waitBusy();
        }
    }

    void initHardwareAfterWake()
    {
        pinMode(nssPin, OUTPUT);
        digitalWrite(nssPin, HIGH);

        pinMode(busyPin, INPUT);
        pinMode(dio1Pin, INPUT);

        // No reset!
        waitBusy();
    }




    // -------------------------------------------------
    // SX1262 Commands
    // -------------------------------------------------
	
	void setStandby()
    {
        uint8_t p[1] = {0x00};

        spiWriteCmd(
            0x80,     // SetStandby
            p,
            1
        );
    }



    void setRegulatorMode(bool dcDc)
    {

        uint8_t p[1];

        p[0] = dcDc ? 0x01 : 0x00;

        spiWriteCmd(
            0x96,     // SetRegulatorMode
            p,
            1
        );
    }



    void setPacketType(uint8_t type = 0x01)
    {

        uint8_t p[1] = {type};

        spiWriteCmd(
            0x8A,
            p,
            1
        );
    }



    void setRfFrequency(uint32_t freqHz)
    {

        uint32_t frf =
        (uint32_t)((double)freqHz *
        33554432.0 /
        32000000.0);


        uint8_t p[4];

        p[0] = frf >> 24;
        p[1] = frf >> 16;
        p[2] = frf >> 8;
        p[3] = frf;


        spiWriteCmd(
            0x86,
            p,
            4
        );
    }



    void calibrateImage(uint32_t freqHz)
    {

        uint8_t p[2];


        if(freqHz >= 902000000 &&
           freqHz <= 928000000)
        {
            p[0]=0xE1;
            p[1]=0xE9;
        }
        else
        {
            return;
        }


        spiWriteCmd(
            0x98,
            p,
            2
        );
    }



    void setModulationParams(uint8_t sf,
                             uint8_t bw,
                             uint8_t cr,
                             uint8_t ldro=0)
    {

        uint8_t p[4];

        p[0]=sf;
        p[1]=bw;
        p[2]=cr;
        p[3]=ldro;


        spiWriteCmd(
            0x8B,
            p,
            4
        );
    }
	
	
    void setPacketParams(uint16_t preambleLen,
                         uint8_t headerType,
                         uint8_t payloadLen,
                         uint8_t crc,
                         uint8_t invertIq)
    {

        uint8_t p[6];

        p[0] = (uint8_t)(preambleLen >> 8);
        p[1] = (uint8_t)(preambleLen & 0xFF);

        p[2] = headerType;
        p[3] = payloadLen;
        p[4] = crc;
        p[5] = invertIq;


        spiWriteCmd(
            0x8C,       // SetPacketParams
            p,
            6
        );
    }



    void setBufferBaseAddress(uint8_t txBase,
                              uint8_t rxBase)
    {

        uint8_t p[2];

        p[0] = txBase;
        p[1] = rxBase;


        spiWriteCmd(
            0x8F,
            p,
            2
        );
    }



    void setDioIrqParams(uint16_t irqMask,
                         uint16_t dio1Mask)
    {

        uint8_t p[8];


        p[0] = irqMask >> 8;
        p[1] = irqMask & 0xFF;


        p[2] = dio1Mask >> 8;
        p[3] = dio1Mask & 0xFF;


        // DIO2 / DIO3 masks
        p[4] = 0x00;
        p[5] = 0x00;
        p[6] = 0x00;
        p[7] = 0x00;


        spiWriteCmd(
            0x08,
            p,
            8
        );
    }



    void clearIrqStatus(uint16_t mask = 0xFFFF)
    {

        uint8_t p[2];


        p[0] = mask >> 8;
        p[1] = mask & 0xFF;


        spiWriteCmd(
            0x02,
            p,
            2
        );
    }



    uint16_t getIrqStatus()
    {
        return spiReadCmd16(0x12);
    }



    // -------------------------------------------------
    // Wake On Radio / RX Duty Cycle
    // -------------------------------------------------

    void startReceiveDutyCycle(uint32_t rxPeriod,
                               uint32_t sleepPeriod)
    {

        uint8_t p[6];


        p[0] = rxPeriod >> 16;
        p[1] = rxPeriod >> 8;
        p[2] = rxPeriod;


        p[3] = sleepPeriod >> 16;
        p[4] = sleepPeriod >> 8;
        p[5] = sleepPeriod;


        spiWriteCmd(
            0x94,       // SetRxDutyCycle
            p,
            6
        );
    }



    void startReceive(uint32_t timeout = 0)
    {

        uint8_t p[3];


        p[0] = timeout >> 16;
        p[1] = timeout >> 8;
        p[2] = timeout;


        spiWriteCmd(
            0x82,       // SetRx
            p,
            3
        );
    }



    // -------------------------------------------------
    // TX helpers
    // -------------------------------------------------

    void writeBuffer(uint8_t offset,
                     const uint8_t *data,
                     uint8_t length)
    {

        waitBusy();

        digitalWrite(nssPin, LOW);


        SPI.transfer(0x0E);

        SPI.transfer(offset);


        for(uint8_t i=0;i<length;i++)
        {
            SPI.transfer(data[i]);
        }


        digitalWrite(nssPin, HIGH);

        waitBusy();
    }



    void startTx(uint32_t timeout = 0)
    {

        uint8_t p[3];

        p[0] = timeout >> 16;
        p[1] = timeout >> 8;
        p[2] = timeout;


        spiWriteCmd(
            0x83,
            p,
            3
        );
    }



    // -------------------------------------------------
    // Convenience initialization
    // -------------------------------------------------

    void beginLoRa915()
    {

        initHardware(true);

        delay(10);

        setStandby();

        // Ebyte boards normally use the DC-DC path
        setRegulatorMode(true);

        setPacketType(0x01);

        setRfFrequency(915000000);

        calibrateImage(915000000);


        // SF7 / BW125 / CR 4/5
        setModulationParams(
            0x07,
            0x04,
            0x01,
            0x00
        );


        // RX packet configuration
        setPacketParams(
            12,
            0x00,
            0xFF,
            0x01,
            0x00
        );


        setBufferBaseAddress(
            0x00,
            0x00
        );


        clearIrqStatus();


        // Preamble detected + Header valid
        setDioIrqParams(
            0x000C,
            0x000C
        );
    }


};

#endif
