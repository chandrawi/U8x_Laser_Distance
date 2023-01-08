#include <U8xLaser.h>

#if defined(Serial)
U8xLaser::U8xLaser(int8_t pwrEn, int8_t reset)
{
    U8xLaser(Serial, pwrEn, reset);
}
#endif

U8xLaser::U8xLaser(HardwareSerial& serial, int8_t pwrEn, int8_t reset)
{
    _serial = &serial;
    _pwrEn = pwrEn;
    _reset = reset;
}
