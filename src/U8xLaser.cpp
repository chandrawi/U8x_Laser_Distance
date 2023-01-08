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
    _address = 0;
}

void U8xLaser::begin(uint32_t baud)
{
    if (_pwrEn > -1) {
        pinMode(_pwrEn, OUTPUT);
        digitalWrite(_pwrEn, HIGH);
    }
    if (_reset > -1) {
        pinMode(_reset, OUTPUT);
    }
    _serial->begin(baud);
    // Send auto baudrate command
    delay(100);
    _serial->write(U8X_AUTO_BAUD_RATE);
    // Try to get address from U8x
    uint32_t t = millis();
    while (!_serial->available()) {
        if (millis() - t > U8X_TIMEOUT_CMD) return;
    }
    _address = _serial->read();
}

void U8xLaser::end()
{
    if (_pwrEn > -1) digitalWrite(_pwrEn, LOW);
    _serial->end();
}

void U8xLaser::sendFrame(U8xFrame_t* frame)
{
    // Send head, address and register
    _serial->write(U8X_HEAD);
    _serial->write(frame->addr);
    _serial->write(frame->regH);
    _serial->write(frame->regL);
    // Only send payload and payload count if payload count > 0
    if (frame->size > 0) {
        _serial->write(0x00);
        _serial->write(frame->size);
        for (uint8_t i=0; i<(frame->size*2); i++) {
            _serial->write(frame->payload[i]);
        }
    }
    // Send calculated checksum
    _serial->write(checksum(frame));
}

bool U8xLaser::receiveFrame(U8xFrame_t* frame, uint32_t timeout)
{
    uint32_t t = millis();
    // Find command frame head or error head
    while (millis() - t < timeout) {
        if (_serial->available()) {
            uint8_t head = _serial->read();
            if (head == U8X_HEAD || head == U8X_HEAD_ERROR) break;
        }
    }
    // Wait for incoming data then get address, register and payload count
    while (_serial->available() < 5) {
        if (millis() - t > timeout) return false;
    }
    frame->addr = _serial->read();
    frame->regH = _serial->read();
    frame->regL = _serial->read();
    _serial->read();
    frame->size = _serial->read();
    // Payload size in bytes is twice of received payload count with maximum 6 bytes
    frame->size = frame->size > U8X_MAX_PAYLOAD ? U8X_MAX_PAYLOAD : frame->size;
    // Wait for incoming data then get the payloads
    while (_serial->available() < ((frame->size*2)+1)) {
        if (millis() - t > timeout) return false;
    }
    for (uint8_t i=0; i<(frame->size*2); i++) {
        frame->payload[i] = _serial->read();
    }
    // Check received checksum
    if (_serial->read() != checksum(frame)) return false;
    return true;
}

uint8_t U8xLaser::checksum(U8xFrame_t* frame)
{
    uint8_t sum = frame->addr + frame->regH + frame->regL;
    if (frame->size > 0) {    
        sum += frame->size;
        for (uint8_t i=0; i<(frame->size*2); i++) {
            sum += frame->payload[i];
        }
    }
    return sum;
}
