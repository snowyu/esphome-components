#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"


namespace esphome {
namespace amg88xx {

#define byte uint8_t

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define AMG88XX_ADDRESS (0x69)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum {
  // Power Control Register
  AMG88XX_PCTL = 0x00,
  // Reset Register
  AMG88XX_RST = 0x01,
  // Frame Rate Register
  AMG88XX_FPSC = 0x02,
  // nterrupt Control Register
  AMG88XX_INTC = 0x03,
  // Status Register
  AMG88XX_STAT = 0x04,
  // Status Clear Register
  AMG88XX_SCLR = 0x05,
  // 0x06 reserved
  // Average Register
  AMG88XX_AVE = 0x07,
  // Interrupt Level Register
  // Interrupt Level upper limit setting: 0-7bit
  AMG88XX_INTHL = 0x08,
  // Interrupt Level upper limit setting: 8-11bit
  AMG88XX_INTHH = 0x09,
  // Interrupt Level lower limit setting: 0-7bit
  AMG88XX_INTLL = 0x0A,
  // Interrupt Level lower limit setting: 8-11bit
  AMG88XX_INTLH = 0x0B,
  // Setting of Interrupt Hysteresis Level: 0-7bit
  AMG88XX_IHYSL = 0x0C,
  // Setting of Interrupt Hysteresis Level: 8-11bit
  AMG88XX_IHYSH = 0x0D,
  // Thermistor Register: 0-7bit
  AMG88XX_TTHL = 0x0E,
  // Thermistor Register: 8-10bit, 11bit is sign bit
  AMG88XX_TTHH = 0x0F,
  // Interrupt Table Register: 0x10-0x17
  AMG88XX_INT_OFFSET = 0x010,
  // Temperature Table Register: 0x80-0xFF  total 64(8x8) data
  AMG88XX_PIXEL_OFFSET = 0x80
};

enum PowerMode {
  AMG88XX_NORMAL_MODE = 0x00,
  AMG88XX_SLEEP_MODE = 0x01,
  // Stand-by mode (60sec intermittence)
  AMG88XX_STAND_BY_60 = 0x20,
  // Stand-by mode (10sec intermittence)
  AMG88XX_STAND_BY_10 = 0x21
};

enum SWReset { AMG88XX_FLAG_RESET = 0x30, AMG88XX_INITIAL_RESET = 0x3F };

enum FrameRate { AMG88XX_FPS_10 = 0x00, AMG88XX_FPS_1 = 0x01 };

enum IntEnable { AMG88XX_INT_DISABLED = 0x00, AMG88XX_INT_ENABLED = 0x01 };

enum IntMode { AMG88XX_DIFFERENCE = 0x00, AMG88XX_ABSOLUTE_VALUE = 0x01 };

/*=========================================================================*/

#define AMG88XX_PIXEL_ARRAY_SIZE 64
#define AMG88XX_PIXEL_TEMP_CONVERSION .25
#define AMG88XX_THERMISTOR_CONVERSION .0625
#define AMG88XX_DEFAULT_HYSTERESIS_FACTOR .95
#define AMG88XX_NULL_HYSTERESIS INFINITY

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with AMG88XX
   IR sensor chips
*/
/**************************************************************************/

class AMG88XXComponent : public PollingComponent, public i2c::I2CDevice {
public:
  float get_setup_priority() const override { return esphome::setup_priority::BUS; }
  void setup() override;
  void dump_config() override;
  void update() override;

  float readThermistor();

  void setMovingAverageMode(bool mode);

  void enableInterrupt();
  void disableInterrupt();
  void setInterruptMode(uint8_t mode);
  void getInterrupt(uint8_t *buf, uint8_t size = 8);
  void clearInterrupt();

  void setFrameRate(FrameRate fps);
  void setPowerMode(PowerMode mode);
  void reset(SWReset flag);

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_ir_camera_sensor(text_sensor::TextSensor *ir_camera_sensor) { ir_camera_sensor_ = ir_camera_sensor; }
  void init_addr_select_pin(GPIOPin *addr_select_pin) { this->addr_select_pin_ = addr_select_pin; }
  void init_int_pin(GPIOPin *int_pin) { this->int_pin_ = int_pin; }
  void init_frame_rate(FrameRate fps) { this->frame_rate_ = fps; }
  void init_power_mode(PowerMode mode) { this->power_mode_ = mode; }

  void readPixelsRaw(uint16_t *buf, uint8_t pixels = AMG88XX_PIXEL_ARRAY_SIZE);
  void readPixels(float *buf, uint8_t pixels = AMG88XX_PIXEL_ARRAY_SIZE);
  // this will automatically set hysteresis to 95% of the high value
  void setInterruptLevels(float high, float low);

  // this will manually set hysteresis
  void setInterruptLevels(float high, float low, float hysteresis);


protected:
  // AD_SELECT(address select) pin
  GPIOPin *addr_select_pin_{nullptr};
  // INT(Interrupt flag) pin
  GPIOPin *int_pin_{nullptr};
  PowerMode power_mode_{AMG88XX_NORMAL_MODE};
  FrameRate frame_rate_{AMG88XX_FPS_10};
  float int_low_{20};
  float int_high_{80};
  float int_hysteresis_{80*AMG88XX_DEFAULT_HYSTERESIS_FACTOR};
  sensor::Sensor *temperature_sensor_;
  text_sensor::TextSensor *ir_camera_sensor_;

  // // read a given register
  // virtual bool read_reg(uint8_t reg, uint8_t *value);
  // // write a value to a given register
  // virtual bool write_reg(uint8_t reg, uint8_t value);
  // // update registers with given pin value.
  // virtual void update_reg(uint8_t pin, bool pin_value, uint8_t reg_a);

  void write8(byte reg, byte value);
  void write16(byte reg, uint16_t value);
  uint8_t read8(byte reg);

  bool _read(uint8_t reg, uint8_t *buf, uint8_t num, bool stop = true);
  bool _write(uint8_t reg, uint8_t *buf, uint8_t num, bool stop = true);

  void update_device_temperature();
  void update_ir_camera();

private:
  uint16_t _raw_pixels[AMG88XX_PIXEL_ARRAY_SIZE];
  float _device_temperature{100};
  // uint32_t _last_time;

  uint16_t getPixelTemperatureRaw(unsigned char pixelAddr);

  float signedMag12ToFloat(uint16_t val);
  float int12ToFloat(uint16_t val);

  // The power control register
  struct pctl {
    // 0x00 = Normal Mode
    // 0x01 = Sleep Mode
    // 0x20 = Stand-by mode (60 sec intermittence)
    // 0x21 = Stand-by mode (10 sec intermittence)

    uint8_t PCTL : 8;

    uint8_t get() { return PCTL; }
  };
  pctl _pctl;

  // reset register
  struct ResetStruc {
    // 0x30 = flag reset (all clear status reg 0x04, interrupt flag and
    // interrupt table) 0x3F = initial reset (brings flag reset and returns to
    // initial setting)

    uint8_t RST : 8;

    uint8_t get() { return RST; }
  };

  // frame rate register
  struct FPSStruc {

    // 0 = 10FPS
    // 1 = 1FPS
    uint8_t FPS : 1;

    uint8_t get() { return FPS & 0x01; }
  };

  // interrupt control register
  struct intc {

    // 0 = INT output reactive (Hi-Z)
    // 1 = INT output active
    uint8_t INTEN : 1;

    // 0 = Difference interrupt mode
    // 1 = absolute value interrupt mode
    uint8_t INTMOD : 1;

    uint8_t get() { return (INTMOD << 1 | INTEN) & 0x03; }
  };
  intc _intc;

  // status register
  struct stat {
    uint8_t unused : 1;
    // interrupt outbreak (val of interrupt table reg)
    uint8_t INTF : 1;

    // temperature output overflow (val of temperature reg)
    uint8_t OVF_IRS : 1;

    // thermistor temperature output overflow (value of thermistor)
    uint8_t OVF_THS : 1;

    uint8_t get() {
      return ((OVF_THS << 3) | (OVF_IRS << 2) | (INTF << 1)) & 0x0E;
    }
  };
  stat _stat;

  // status clear register
  // write to clear overflow flag and interrupt flag
  // after writing automatically turns to 0x00
  struct sclr {
    uint8_t unused : 1;
    // interrupt flag clear
    uint8_t INTCLR : 1;
    // temp output overflow flag clear
    uint8_t OVS_CLR : 1;
    // thermistor temp output overflow flag clear
    uint8_t OVT_CLR : 1;

    uint8_t get() {
      return ((OVT_CLR << 3) | (OVS_CLR << 2) | (INTCLR << 1)) & 0x0E;
    }
  };
  sclr _sclr;

  // average register
  // for setting moving average output mode
  struct ave {
    uint8_t unused : 5;
    // 1 = twice moving average mode
    uint8_t MAMOD : 1;

    uint8_t get() { return (MAMOD << 5); }
  };
  struct ave _ave;

  // interrupt level registers
  // for setting upper / lower limit hysteresis on interrupt level

  // interrupt level upper limit setting. Interrupt output
  // and interrupt pixel table are set when value exceeds set value
  struct inthl {
    uint8_t INT_LVL_H : 8;

    uint8_t get() { return INT_LVL_H; }
  };
  struct inthl _inthl;

  struct inthh {
    uint8_t INT_LVL_H : 4;

    uint8_t get() { return INT_LVL_H; }
  };
  struct inthh _inthh;

  // interrupt level lower limit. Interrupt output
  // and interrupt pixel table are set when value is lower than set value
  struct intll {
    uint8_t INT_LVL_L : 8;

    uint8_t get() { return INT_LVL_L; }
  };
  struct intll _intll;

  struct intlh {
    uint8_t INT_LVL_L : 4;

    uint8_t get() { return (INT_LVL_L & 0xF); }
  };
  struct intlh _intlh;

  // setting of interrupt hysteresis level when interrupt is generated.
  // should not be higher than interrupt level
  struct ihysl {
    uint8_t INT_HYS : 8;

    uint8_t get() { return INT_HYS; }
  };
  struct ihysl _ihysl;

  struct ihysh {
    uint8_t INT_HYS : 4;

    uint8_t get() { return (INT_HYS & 0xF); }
  };
  struct ihysh _ihysh;

  // thermistor register
  // SIGNED MAGNITUDE FORMAT
  struct tthl {
    uint8_t TEMP : 8;

    uint8_t get() { return TEMP; }
  };
  struct tthl _tthl;

  struct tthh {
    uint8_t TEMP : 3;
    uint8_t SIGN : 1;

    uint8_t get() { return ((SIGN << 3) | TEMP) & 0xF; }
  };
  struct tthh _tthh;

  // temperature registers 0x80 - 0xFF
  /*
  //read to indicate temperature data per 1 pixel
  //SIGNED MAGNITUDE FORMAT
  struct t01l {
          uint8_t TEMP : 8;

          uint8_t get(){
                  return TEMP;
          }
  };
  struct t01l _t01l;

  struct t01h {
          uint8_t TEMP : 3;
          uint8_t SIGN : 1;

          uint8_t get(){
                  return ( (SIGN << 3) | TEMP) & 0xF;
          }
  };
  struct t01h _t01h;
  */

};

}  // namespace amg88xx
}  // namespace esphome
