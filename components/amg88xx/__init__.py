import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_INPUT,
    CONF_NUMBER,
    CONF_MODE,
    CONF_INVERTED,
    CONF_INTERRUPT,
    CONF_OPEN_DRAIN_INTERRUPT,
    CONF_OUTPUT,
    CONF_PIN,
    CONF_PULLUP,
    CONF_TEMPERATURE,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from esphome.cpp_helpers import setup_entity

CODEOWNERS = ["@riceball"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["amg88xx", "i2c", "sensor", "text_sensor"]

amg88xx_base_ns = cg.esphome_ns.namespace("amg88xx")
AMG88XX = amg88xx_base_ns.class_("AMG88XXComponent", cg.PollingComponent, i2c.I2CDevice)
AMG88XXPowerMode = amg88xx_base_ns.enum("PowerMode")
AMG88XXFameRate = amg88xx_base_ns.enum("FrameRate")

CONF_AMG88XX_ID = "amg88xx"
CONF_ADDR_SELECT_PIN = "addr_select_pin"
CONF_POWER_MODE = "power_mode"
CONF_TRIGGER = "trigger"
CONF_LOW = "low"
CONF_HIGH = "high"
CONF_HYSTERESIS = "hysteresis"
CONF_FRAME_RATE = "frame_rate"
CONF_DEVICE_TEMPERATURE =  "device_temperature"
CONF_IR_CAMERA =  "ir_camera"

temperature_range_param = cv.float_range(min=-20, max=100)

AMG88XX_POWER_MODE = {
    "NORMAL": AMG88XXPowerMode.AMG88XX_NORMAL_MODE,
    "STAND_BY": AMG88XXPowerMode.AMG88XX_STAND_BY_10,
    "STAND_BY_10s": AMG88XXPowerMode.AMG88XX_STAND_BY_10,
    "STAND_BY_60s": AMG88XXPowerMode.AMG88XX_STAND_BY_60,
    "STAND_BY_1m": AMG88XXPowerMode.AMG88XX_STAND_BY_60,
    "SLEEP": AMG88XXPowerMode.AMG88XX_SLEEP_MODE,
}

AMG88XX_FRAME_RATE = {
    "10FPS": AMG88XXFameRate.AMG88XX_FPS_10,
    "10 FPS": AMG88XXFameRate.AMG88XX_FPS_10,
    "1FPS": AMG88XXFameRate.AMG88XX_FPS_1,
    "1 FPS": AMG88XXFameRate.AMG88XX_FPS_1,
}

# export to esphome
CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(AMG88XX),
        cv.Optional(CONF_ADDR_SELECT_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_POWER_MODE, default="NORMAL"): cv.enum(
            AMG88XX_POWER_MODE, upper=True
        ),
        cv.Optional(CONF_FRAME_RATE, default="10FPS"): cv.enum(
            AMG88XX_FRAME_RATE, upper=True
        ),
        cv.Optional(CONF_TRIGGER): cv.Schema(
            {
                cv.Required(CONF_PIN): pins.internal_gpio_input_pin_number,
                cv.Optional(CONF_LOW, default=0): temperature_range_param,
                cv.Optional(CONF_HIGH, default=80): temperature_range_param,
                cv.Optional(CONF_HYSTERESIS): temperature_range_param,
            }
        ),
        cv.Optional(CONF_DEVICE_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_IR_CAMERA): text_sensor.text_sensor_schema(
        ),
    })
    .extend(cv.polling_component_schema("100ms"))
    .extend(i2c.i2c_device_schema(0x69))
)

SETTERS = {
    # pin assignment
    CONF_ADDR_SELECT_PIN: "init_addr_select_pin",
}

# export to esphome
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for key, setter in SETTERS.items():
        if key in config:
            cg.add(getattr(var, setter)(config[key]))

    triggerMode = CONF_TRIGGER in config and config[CONF_TRIGGER]
    if triggerMode and CONF_PIN in triggerMode:
        pin = await cg.gpio_pin_expression(triggerMode[CONF_PIN])
        cg.add(var.init_int_pin(pin))

    if CONF_DEVICE_TEMPERATURE in config:
        conf = config[CONF_DEVICE_TEMPERATURE]
        if CONF_NAME not in conf:
            conf[CONF_NAME] = CONF_AMG88XX_ID + " device temperature"
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_temperature_sensor(sens))

    if CONF_IR_CAMERA in config:
        conf = config[CONF_IR_CAMERA]
        if CONF_NAME not in conf:
            conf[CONF_NAME] = CONF_AMG88XX_ID + " ir camera"
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_ir_camera_sensor(sens))

