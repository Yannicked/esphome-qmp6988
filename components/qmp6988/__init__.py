import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID
)

AUTO_LOAD = ['sensor']
CODEOWNERS = ["@yannicked"]
DEPENDENCIES = ["i2c"]

CONF_QMP6988_ID = "qmp6988_id"

qmp6988_ns = cg.esphome_ns.namespace("qmp6988")

QMP6988Component = qmp6988_ns.class_(
    "QMP6988Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(QMP6988Component)
    }
).extend(cv.polling_component_schema("60s")).extend(i2c.i2c_device_schema(0x56))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
