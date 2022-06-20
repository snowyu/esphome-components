# Custom Hardware Components for ESPHome

## Thermal Camera - AMG88XX

### AMG88XX Specs

* AMG88XX
  * Pixel number: 64 (8x8 Matrix)
  * External Interface: I2C
    * Number of Sensor Addresses: 2（I2C Slave Address）
    * The Default Address is 0x69. the address will be `0x68` if `addr_select_pin` is GND, or it's the `0x69`.
  * Frame Rate: 10fps/1fps
  * Power Mode:
    * Normal
    * Sleep
    * Stand-by: （10sec or 60sec interval）
  * Calculate Mode
    * No moving average
    * Twice moving average
  * Temperature Output Resolution: 0.25℃
  * Temperature Range of Measuring Object: 0℃～80℃(High gain), -20℃～100℃(Low gain)
  * Temperature Accuracy: ±2.5℃(High gain), ±3.0℃(Low gain)
  * Field of View: 60°（Horizontal, Vertical)
  * Rated detection distance: 5m（Max.）
  * Get Device Temperature

### ESPHome Configuration Options

* addr_select_pin(INPUT): optional
* power_mode(optional): defaults to "NORMAL"
  * "NORMAL"
  * "STAND_BY": same as "STAND_BY_10s"
  * "STAND_BY_10s"
  * "STAND_BY_60s"
  * "STAND_BY_1m"
  * "SLEEP"
* frame_rate(optional): defaults to 1 fps
  * "10FPS" or "10 FPS"
  * "1FPS" or "1 FPS"
* trigger_mode: only available when the `pin`(INT) is set
  * pin: OUTPUT pin.
  * low: the lowest temperature to trigger
  * high: the highest temperature to trigger
  * hysteresis: defaults to high * 0.95
* device_temperature(optional): enable the device temperature sensor. defaults to no sensor.
  * name: defaults to `amg88xx device temperature`
  * accuracy_decimals: defaults to 2
* ir_camera(optional): enable the ir camera sensor. defaults to no sensor.
  * name: defaults to `amg88xx ir camera`
  * Use `the ugly text sensor`(howto replace it) to pass the 8x8 raw temperature array currently.
  * So you should disable the recorder in the HA `configuration.yaml`:

  ```yml
  recorder:
    exclude:
      entity_globs:
        - sensor.*_ir_camera
  history:
    exclude:
      entity_globs:
        - sensor.*_ir_camera
  ```

### Sample ESPHome Configuration

```yml
substitutions:
  # Name the device and it's entities
  device: amg8833
  device_name: amg8833_1

esphome:
  name: $device_name
  platform: ESP32
  board: esp32doit-devkit-v1

external_components:
  - source: github://snowyu/esphome-components

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  reboot_timeout: 20min

  # Enable fallback hotspot in case wifi connection fails
  ap:
    ssid: "$device_name Fallback Hotspot"
    password: !secret ap_password

logger:

api:

ota:
  password: !secret ota_password
  reboot_timeout: 2min

i2c:
#   sda: 21
#   scl: 22
#   scan: true

amg88xx:
  ir_camera:
    name: $device_name ir camera
  device_temperature:
    name: $device_name device temperature
```

### IR Camera Usage

