#ifndef _U8X_LASER_H_
#define _U8X_LASER_H_

#include <Arduino.h>
#include <HardwareSerial.h>

#define U8X_HEAD                    0xAA
#define U8X_HEAD_ERROR              0xEE
#define U8X_ADDRESS                 0x00

#define U8X_REG_ERR_CODE            0x0000
#define U8X_REG_BAT_VLTG            0x0006
#define U8X_REG_HW_VER              0x000A
#define U8X_REG_SW_VER              0x000C
#define U8X_REG_SER_NUM             0x000E
#define U8X_REG_ADDRESS             0x0010
#define U8X_REG_OFFSET              0x0012
#define U8X_REG_MEA_START           0x0020
#define U8X_REG_MEA_RESULT          0x0022
#define U8X_REG_CTRL_LD             0x01BE

#define U8X_STATUS_NO_ERR           0x0000
#define U8X_STATUS_VLTG_LOW         0x0001
#define U8X_STATUS_INT_ERR          0x0002
#define U8X_STATUS_TEMP_LOW         0x0003
#define U8X_STATUS_TEMP_HIGH        0x0004
#define U8X_STATUS_OUT_RANGE        0x0005
#define U8X_STATUS_INV_RESULT       0x0006
#define U8X_STATUS_BG_LIGHT         0x0007
#define U8X_STATUS_LASER_WEAK       0x0008
#define U8X_STATUS_LASER_STRONG     0x0009
#define U8X_STATUS_HW_FAULT_1       0x000A
#define U8X_STATUS_HW_FAULT_2       0x000B
#define U8X_STATUS_HW_FAULT_3       0x000C
#define U8X_STATUS_HW_FAULT_4       0x000D
#define U8X_STATUS_HW_FAULT_5       0x000E
#define U8X_STATUS_LASER_UNSTABLE   0x000F
#define U8X_STATUS_HW_FAULT_6       0x0010
#define U8X_STATUS_HW_FAULT_7       0x0011
#define U8X_STATUS_INV_FRAME        0x0081

#define U8X_AUTO_BAUD_RATE          0x55
#define U8X_STOP_MEASURE            0x58

#define U8X_BAUD_RATE               119200
#define U8X_TIMEOUT_CMD             100
#define U8X_TIMEOUT_MEA_FAST        1000
#define U8X_TIMEOUT_MEA_SLOW        5000
#define U8X_MAX_PAYLOAD             6

typedef struct {
    uint8_t addr;
    uint8_t regH;
    uint8_t regL;
    uint8_t size;
    uint8_t payload[U8X_MAX_PAYLOAD];
} U8xFrame_t;

class U8xLaser
{
    public:

#if defined(Serial)
        U8xLaser(int8_t pwrEn=-1, int8_t reset=-1);
#endif
        U8xLaser(HardwareSerial& serial, int8_t pwrEn=-1, int8_t reset=-1);

        void begin(uint32_t baud=U8X_BAUD_RATE);
        void end();

        uint16_t status();
        uint16_t inputVoltage();
        uint16_t hardwareVersion();
        uint16_t softwareVersion();
        uint32_t serialNumber();

        void sendFrame(U8xFrame_t* frame);
        bool receiveFrame(U8xFrame_t* frame, uint32_t timeout=U8X_TIMEOUT_CMD);
        uint8_t checksum(U8xFrame_t* frame);

    private:

        U8xFrame_t _frame;
        HardwareSerial* _serial;
        int8_t _pwrEn, _reset;
        uint8_t _address;
};

#endif