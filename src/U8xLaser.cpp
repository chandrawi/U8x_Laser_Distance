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

uint16_t U8xLaser::status()
{
    _frame.addr = 0x80 | _address;
    _frame.regH = U8X_REG_ERR_CODE >> 8;
    _frame.regL = U8X_REG_ERR_CODE;
    _frame.size = 0;
    sendFrame(&_frame);
    receiveFrame(&_frame);
    return (_frame.payload[0] << 8) | _frame.payload[1];
}

uint16_t U8xLaser::inputVoltage()
{
    _frame.addr = 0x80 | _address;
    _frame.regH = U8X_REG_BAT_VLTG >> 8;
    _frame.regL = U8X_REG_BAT_VLTG;
    _frame.size = 0;
    sendFrame(&_frame);
    receiveFrame(&_frame);
    return (_frame.payload[0] << 8) | _frame.payload[1];
}

uint16_t U8xLaser::hardwareVersion()
{
    _frame.addr = 0x80 | _address;
    _frame.regH = U8X_REG_HW_VER >> 8;
    _frame.regL = U8X_REG_HW_VER;
    _frame.size = 0;
    sendFrame(&_frame);
    receiveFrame(&_frame);
    return (_frame.payload[0] << 8) | _frame.payload[1];
}

uint16_t U8xLaser::softwareVersion()
{
    _frame.addr = 0x80 | _address;
    _frame.regH = U8X_REG_SW_VER >> 8;
    _frame.regL = U8X_REG_SW_VER;
    _frame.size = 0;
    sendFrame(&_frame);
    receiveFrame(&_frame);
    return (_frame.payload[0] << 8) | _frame.payload[1];
}

uint32_t U8xLaser::serialNumber()
{
    _frame.addr = 0x80 | _address;
    _frame.regH = U8X_REG_SER_NUM >> 8;
    _frame.regL = U8X_REG_SER_NUM;
    _frame.size = 0;
    sendFrame(&_frame);
    receiveFrame(&_frame);
    return (_frame.payload[0] << 24) | (_frame.payload[1] << 16) | (_frame.payload[2] << 8) | _frame.payload[3];
}

int32_t U8xLaser::measureResult()
{
    _frame.addr = 0x80 | _address;
    _frame.regH = U8X_REG_MEA_RESULT >> 8;
    _frame.regL = U8X_REG_MEA_RESULT;
    _frame.size = 0;
    sendFrame(&_frame);
    receiveFrame(&_frame, U8X_TIMEOUT_MEA_SLOW);
    _signal_quality = (_frame.payload[4] << 8) + _frame.payload[5];
    return (_frame.payload[0] << 24) | (_frame.payload[1] << 16) | (_frame.payload[2] << 8) | _frame.payload[3];
}

int32_t U8xLaser::measureSingle()
{
    _startMeasure(0x00);
    receiveFrame(&_frame, U8X_TIMEOUT_MEA_SLOW);
    _signal_quality = (_frame.payload[4] << 8) + _frame.payload[5];
    return (_frame.payload[0] << 24) | (_frame.payload[1] << 16) | (_frame.payload[2] << 8) | _frame.payload[3];
}

int32_t U8xLaser::measureSingleSlow()
{
    _startMeasure(0x01);
    receiveFrame(&_frame, U8X_TIMEOUT_MEA_SLOW);
    _signal_quality = (_frame.payload[4] << 8) + _frame.payload[5];
    return (_frame.payload[0] << 24) | (_frame.payload[1] << 16) | (_frame.payload[2] << 8) | _frame.payload[3];
}

int32_t U8xLaser::measureSingleFast()
{
    _startMeasure(0x02);
    receiveFrame(&_frame, U8X_TIMEOUT_MEA_FAST);
    _signal_quality = (_frame.payload[4] << 8) + _frame.payload[5];
    return (_frame.payload[0] << 24) | (_frame.payload[1] << 16) | (_frame.payload[2] << 8) | _frame.payload[3];
}

void U8xLaser::startMeasure()
{
    _startMeasure(0x04);
}

void U8xLaser::startMeasureSlow()
{
    _startMeasure(0x05);
}

void U8xLaser::startMeasureFast()
{
    _startMeasure(0x06);
}

void U8xLaser::stopMeasure()
{
    _serial->write(U8X_STOP_MEASURE);
}

uint16_t U8xLaser::getSignalQuality()
{
    return _signal_quality;
}

void U8xLaser::laserOn()
{
    _laserControl(0x01);
}

void U8xLaser::laserOff()
{
    _laserControl(0x00);
}

void U8xLaser::_startMeasure(uint8_t measureType)
{
    _frame.addr = _address;
    _frame.regH = U8X_REG_MEA_START >> 8;
    _frame.regL = U8X_REG_MEA_START;
    _frame.size = 1;
    _frame.payload[0] = 0x00;
    _frame.payload[1] = measureType;
    sendFrame(&_frame);
}

void U8xLaser::_laserControl(uint8_t on_off)
{
    _frame.addr = _address;
    _frame.regH = U8X_REG_CTRL_LD >> 8;
    _frame.regL = U8X_REG_CTRL_LD;
    _frame.size = 1;
    _frame.payload[0] = 0x00;
    _frame.payload[1] = on_off;
    sendFrame(&_frame);
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
