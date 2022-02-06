import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_PRESSURE,
    CONF_ID,
    CONF_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_HECTOPASCAL,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_PRESSURE,
)
from esphome.core import coroutine
from . import QMP6988Component, CONF_QMP6988_ID

DEPENDENCIES = ["qmp6988"]

TYPES = {
    CONF_TEMPERATURE: "set_temperature_sensor",
    CONF_PRESSURE: "set_pressure_sensor"
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(QMP6988Component),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_HECTOPASCAL,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_PRESSURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


@coroutine
def setup_conf(config, key, hub, funcName):
    if key in config:
        conf = config[key]
        var = yield sensor.new_sensor(conf)
        func = getattr(hub, funcName)
        cg.add(func(var))


def to_code(config):
    hub = yield cg.get_variable(config[CONF_QMP6988_ID])
    for key, funcName in TYPES.items():
        yield setup_conf(config, key, hub, funcName)