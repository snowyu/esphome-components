#include <Arduino.h>
#include <base64.h>

#include "amg88xx.h"
#include "esphome/core/log.h"

namespace esphome {
namespace amg88xx {

// #define min(a,b) ((a)<(b)?(a):(b))
// #define max(a,b) ((a)>(b)?(a):(b))
// #define abs(x) ((x)>0?(x):-(x))
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static const char *const TAG = "amg88xx";

static const LogString *power_mode_to_str(PowerMode mode) {
  switch (mode) {
    case AMG88XX_STAND_BY_10:
      return LOG_STR("Stand by(10s)");
    case AMG88XX_STAND_BY_60:
      return LOG_STR("Stand by(60s)");
    case AMG88XX_SLEEP_MODE:
      return LOG_STR("Sleep");
    default:
      return LOG_STR("Normal");
  }
}

/**************************************************************************/
/*!
    @brief  convert a 12-bit integer two's complement value to a int16 number
    @param  val the 12-bit integer  two's complement value to be converted
    @returns the converted int16 value
*/
/**************************************************************************/
inline int16_t int12ToInt16(uint16_t val) {
  int16_t sVal =
      (val << 4); // shift to left so that sign bit of 12 bit integer number is
                  // placed on sign bit of 16 bit signed integer number
  return sVal >> 4; // shift back the signed number
}

void AMG88XXComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AMG88XX...");

  setPowerMode(power_mode_);

  // software reset
  reset(AMG88XX_INITIAL_RESET);

  if (int_pin_ != nullptr) {
    if isinf(int_hysteresis_) int_hysteresis_ = int_high_ * AMG88XX_DEFAULT_HYSTERESIS_FACTOR;
    setInterruptLevels(int_high_, int_low_, int_hysteresis_);
  } else {
    // disable interrupts by default
    disableInterrupt();
  }

  // set to 10 FPS
  setFrameRate(frame_rate_);
  delay(1000);
  _last_time = millis();

  // if (readThermistor() <= 0) {
  //   this->mark_failed();
  // }
}

void AMG88XXComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "AMG88XX:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AMG88XX failed!");
  }
  ESP_LOGCONFIG(TAG, "  Power Mode: %s", LOG_STR_ARG(power_mode_to_str(power_mode_)));
  ESP_LOGCONFIG(TAG, "  Frame rate: %d FPS", this->frame_rate_  == AMG88XX_FPS_10 ? 10 : 1);
  if (this->int_pin_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Trigger Temperature Low: %.2f°C, High: %.2f°C; Hysteresis: %.2f", int_low_, int_high_, int_hysteresis_);
  }
  ESP_LOGCONFIG(TAG, "  Device Temperature: %.2f", this->readThermistor());

  LOG_UPDATE_INTERVAL(this);
  if (this->temperature_sensor_) {
    LOG_SENSOR("  ", "Device Temperature Sensor", this->temperature_sensor_);
  }
  if (this->ir_camera_sensor_) {
    ESP_LOGCONFIG(TAG, "IR Camera Sensor: %s", this->ir_camera_sensor_->get_name().c_str());
  }

}

void AMG88XXComponent::update() {
  this->update_device_temperature();
  this->update_ir_camera();
}

void AMG88XXComponent::update_device_temperature() {
  if (this->temperature_sensor_ != nullptr) {
    float temperature = this->readThermistor();
    if (fabs(temperature - _device_temperature) >= 0.01) {
      this->temperature_sensor_->publish_state(temperature);
      _device_temperature = temperature;
    }
  }
}

void AMG88XXComponent::update_ir_camera() {
  uint32_t cur_time = millis();
  if (this->ir_camera_sensor_ != nullptr && cur_time - _last_time >= 1000) {
    _last_time = cur_time;
    // uint8_t vSize = AMG88XX_PIXEL_ARRAY_SIZE << 1;
    // uint8_t vRaw_pixels[vSize];
    uint16_t vPixels[AMG88XX_PIXEL_ARRAY_SIZE];
    this->readPixelsRaw(vPixels);
    if (memcmp(_raw_pixels, vPixels, AMG88XX_PIXEL_ARRAY_SIZE) != 0) {
      std::string payload;
      for (unsigned char i = 0; i < AMG88XX_PIXEL_ARRAY_SIZE; i++)
      {
        // uint8_t pos = i << 1;
        payload += vPixels[i];

        // ESP_LOGD(TAG, "ir_camera vRaw_pixels[%d].low=%02X, .high=%02X", i, vRaw_pixels[pos], vRaw_pixels[pos+1]);
        // uint16_t t = ((uint16_t)(vRaw_pixels[pos + 1] << 8) | (uint16_t) vRaw_pixels[pos]);
        // if (t == 0) t = 1;
        // ESP_LOGD(TAG, "ir_camera payload[%d]=%04X", i, t);
        // payload += t;
      }
      // ESP_LOGD(TAG, "ir_camera payload length=%zu", payload.length());

      String encoded = base64::encode(payload.c_str());
      this->ir_camera_sensor_->publish_state(encoded.c_str());
      memcpy(_raw_pixels, vPixels, AMG88XX_PIXEL_ARRAY_SIZE);
    }
  }
}

void AMG88XXComponent::setFrameRate(FrameRate fps) {
  FPSStruc v;

  // set to 10 FPS
  v.FPS = fps;
  write8(AMG88XX_FPSC, v.get());
}

void AMG88XXComponent::setPowerMode(PowerMode mode) {
  // enter normal mode
  _pctl.PCTL = mode;
  write8(AMG88XX_PCTL, _pctl.get());
}

void AMG88XXComponent::reset(SWReset flag) {
  ResetStruc v;
  v.RST = flag;
  write8(AMG88XX_RST, v.get());
}

/****************0**********************************************************/
/*!
    @brief  Set the moving average mode.
    @param  mode if True is passed, output will be twice the moving average
*/
/**************************************************************************/
void AMG88XXComponent::setMovingAverageMode(bool mode) {
  _ave.MAMOD = mode;
  write8(AMG88XX_AVE, _ave.get());
}

/**************************************************************************/
/*!
    @brief  Set the interrupt levels. The hysteresis value defaults to .95 *
   high
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
*/
/**************************************************************************/
void AMG88XXComponent::setInterruptLevels(float high, float low) {
  setInterruptLevels(high, low, high * AMG88XX_DEFAULT_HYSTERESIS_FACTOR);
}

/**************************************************************************/
/*!
    @brief  Set the interrupt levels
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
    @param hysteresis the hysteresis value for interrupt detection
*/
/**************************************************************************/
void AMG88XXComponent::setInterruptLevels(float high, float low,
                                          float hysteresis) {
  int highConv = high / AMG88XX_PIXEL_TEMP_CONVERSION;
  highConv = constrain(highConv, -4095, 4095);
  _inthl.INT_LVL_H = highConv & 0xFF;
  _inthh.INT_LVL_H = (highConv & 0x0F00) >> 8;
  this->write8(AMG88XX_INTHL, _inthl.get());
  this->write8(AMG88XX_INTHH, _inthh.get());

  int lowConv = low / AMG88XX_PIXEL_TEMP_CONVERSION;
  lowConv = constrain(lowConv, -4095, 4095);
  _intll.INT_LVL_L = lowConv & 0xFF;
  _intlh.INT_LVL_L = (lowConv & 0x0F00) >> 8;
  this->write8(AMG88XX_INTLL, _intll.get());
  this->write8(AMG88XX_INTLH, _intlh.get());

  int hysConv = hysteresis / AMG88XX_PIXEL_TEMP_CONVERSION;
  hysConv = constrain(hysConv, -4095, 4095);
  _ihysl.INT_HYS = hysConv & 0xFF;
  _ihysh.INT_HYS = (hysConv & 0x0F00) >> 8;
  this->write8(AMG88XX_IHYSL, _ihysl.get());
  this->write8(AMG88XX_IHYSH, _ihysh.get());
}

/**************************************************************************/
/*!
    @brief  enable the interrupt pin on the device.
*/
/**************************************************************************/
void AMG88XXComponent::enableInterrupt() {
  _intc.INTEN = 1;
  this->write8(AMG88XX_INTC, _intc.get());
}

/**************************************************************************/
/*!
    @brief  disable the interrupt pin on the device
*/
/**************************************************************************/
void AMG88XXComponent::disableInterrupt() {
  _intc.INTEN = 0;
  this->write8(AMG88XX_INTC, _intc.get());
}

/**************************************************************************/
/*!
    @brief  Set the interrupt to either absolute value or difference mode
    @param  mode passing AMG88XX_DIFFERENCE sets the device to difference mode,
   AMG88XX_ABSOLUTE_VALUE sets to absolute value mode.
*/
/**************************************************************************/
void AMG88XXComponent::setInterruptMode(uint8_t mode) {
  _intc.INTMOD = mode;
  this->write8(AMG88XX_INTC, _intc.get());
}

/**************************************************************************/
/*!
    @brief  Read the state of the triggered interrupts on the device. The full
   interrupt register is 8 bytes in length.
    @param  buf the pointer to where the returned data will be stored
    @param  size Optional number of bytes to read. Default is 8 bytes.
    @returns up to 8 bytes of data in buf
*/
/**************************************************************************/
void AMG88XXComponent::getInterrupt(uint8_t *buf, uint8_t size) {
  uint8_t bytesToRead = min(size, (uint8_t)8);

  this->_read(AMG88XX_INT_OFFSET, buf, bytesToRead);
}

/**************************************************************************/
/*!
    @brief  Clear any triggered interrupts
*/
/**************************************************************************/
void AMG88XXComponent::clearInterrupt() {
  reset(AMG88XX_FLAG_RESET);
}

/**************************************************************************/
/*!
    @brief  read the onboard thermistor
    @returns a the floating point temperature in degrees Celsius
*/
/**************************************************************************/
float AMG88XXComponent::readThermistor() {
  uint8_t raw[2];
  this->_read(AMG88XX_TTHL, raw, 2);
  uint16_t recast = ((uint16_t)raw[1] << 8) | ((uint16_t)raw[0]);

  return signedMag12ToFloat(recast) * AMG88XX_THERMISTOR_CONVERSION;
}

/**************************************************************************/
/*!
    @brief  Read Infrared sensor raw values
    @param  buf the array to place the pixels in
    @param  pixels Optional number of pixels to read (up to 64). Default is
   64 pixels. Each pixel value is 12 bits, so it is stored in 2 bytes of
   the buf array,
    @return up to 128 bytes of pixel data in buf
*/
/**************************************************************************/
void AMG88XXComponent::readPixelsRaw(uint16_t *buf, uint8_t pixels) {
  uint8_t wordsToRead = min((uint8_t) pixels, (uint8_t) AMG88XX_PIXEL_ARRAY_SIZE);
  for (unsigned char i = 0; i < wordsToRead; i++) {
    buf[i] = this->getPixelTemperatureRaw(i);
  }
}

/**************************************************************************/
/*!
    @brief  Read Infrared sensor values
    @param  buf the array to place the pixels in
    @param  pixels Optional number of pixels to read (up to 64). Default is
   64 pixels.
    @return up to 64 float values of pixel data in buf
*/
/**************************************************************************/
void AMG88XXComponent::readPixels(float *buf, uint8_t pixels) {
  float converted;
  uint8_t wordsToRead = min((uint8_t) pixels, (uint8_t) AMG88XX_PIXEL_ARRAY_SIZE);
  uint16_t rawArray[wordsToRead];
  this->readPixelsRaw(rawArray, wordsToRead);

  for (uint8_t i = 0; i < pixels; i++) {
    converted = int12ToFloat(rawArray[i]) * AMG88XX_PIXEL_TEMP_CONVERSION;
    buf[i] = converted;
  }
}

/**************************************************************************/
/*!
    @brief  write one byte of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
void AMG88XXComponent::write8(byte reg, byte value) {
  this->_write(reg, &value, 1);
}

/**************************************************************************/
/*!
    @brief  read one byte of data from the specified register
    @param  reg the register to read
    @returns one byte of register data
*/
/**************************************************************************/
uint8_t AMG88XXComponent::read8(byte reg) {
  uint8_t ret;
  this->_read(reg, &ret, 1);

  return ret;
}

bool AMG88XXComponent::_read(uint8_t reg, uint8_t *buf, uint8_t num, bool stop) {
  esphome::i2c::ErrorCode errcode = this->read_register(reg, buf, num, stop);
  bool result = errcode == esphome::i2c::ERROR_OK;
  if (!result) this->mark_failed();
  return result;
}

bool AMG88XXComponent::_write(uint8_t reg, uint8_t *buf, uint8_t num, bool stop) {
  if (this->is_failed())
    return false;

  esphome::i2c::ErrorCode errcode = this->write_register(reg, buf, num, stop);
  bool result = errcode == esphome::i2c::ERROR_OK;
  if (!result) this->mark_failed();
  return result;
}

/**************************************************************************/
/*!
    @brief  convert a 12-bit signed magnitude value to a floating point number
    @param  val the 12-bit signed magnitude value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
float AMG88XXComponent::signedMag12ToFloat(uint16_t val) {
  // take first 11 bits as absolute val
  uint16_t absVal = (val & 0x7FF);

  return (val & 0x800) ? 0 - (float)absVal : (float)absVal;
}

/**************************************************************************/
/*!
    @brief  convert a 12-bit integer two's complement value to a floating point
   number
    @param  val the 12-bit integer  two's complement value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
float AMG88XXComponent::int12ToFloat(uint16_t val) {
  return int12ToInt16(val);
}

uint16_t AMG88XXComponent::getPixelTemperatureRaw(unsigned char pixelAddr) {
  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = AMG88XX_PIXEL_OFFSET + (2 * pixelAddr);
  uint8_t buf[2];
  this->_read(pixelLowRegister, buf, 2, false);
  uint16_t result = (uint16_t)buf[1] << 8 | buf[0];
  return result;
}

}  // namespace amg88xx
}  // namespace esphome
