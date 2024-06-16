import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import display
from esphome.const import (
    CONF_ID,
    CONF_WIDTH,
    CONF_LAMBDA,
    CONF_PAGES,
    CONF_CLOCK_PIN,
    CONF_ENABLE_PIN,
    CONF_BRIGHTNESS,
    CONF_ROTATION,
)
CONF_LATCH_PIN = "latch_pin"

CONF_DOUBLE_BUFFER = "double_buffer"

CONF_COLOR_R1 = "r1_pin"
CONF_COLOR_G1 = "g1_pin"
CONF_COLOR_B1 = "b1_pin"
CONF_COLOR_R2 = "r2_pin"
CONF_COLOR_G2 = "g2_pin"
CONF_COLOR_B2 = "b2_pin"
CONF_ADDRESS_A = "addr_a_pin"
CONF_ADDRESS_B = "addr_b_pin"
CONF_ADDRESS_C = "addr_c_pin"
CONF_ADDRESS_D = "addr_d_pin"



# DEPENDENCIES = ["spi"]

CODEOWNERS = ["@IJIJI"]

hub75_ns = cg.esphome_ns.namespace("hub75")

HUB75 = hub75_ns.class_(
    "HUB75", cg.PollingComponent, display.DisplayBuffer
)


CONFIG_SCHEMA = cv.All(
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(HUB75),
            cv.Required(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_CLOCK_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_LATCH_PIN): pins.gpio_output_pin_schema,

            cv.Required(CONF_COLOR_R1): pins.gpio_output_pin_schema,
            cv.Required(CONF_COLOR_G1): pins.gpio_output_pin_schema,
            cv.Required(CONF_COLOR_B1): pins.gpio_output_pin_schema,
            cv.Required(CONF_COLOR_R2): pins.gpio_output_pin_schema,
            cv.Required(CONF_COLOR_G2): pins.gpio_output_pin_schema,
            cv.Required(CONF_COLOR_B2): pins.gpio_output_pin_schema,
            cv.Required(CONF_ADDRESS_A): pins.gpio_output_pin_schema,
            cv.Required(CONF_ADDRESS_B): pins.gpio_output_pin_schema,
            cv.Required(CONF_ADDRESS_C): pins.gpio_output_pin_schema,
            cv.Required(CONF_ADDRESS_D): pins.gpio_output_pin_schema,

            cv.Optional(CONF_WIDTH, default=64): cv.int_,
            # TODO MAKE RANGE
            cv.Optional(CONF_BRIGHTNESS, default=255, ): cv.int_,
            cv.Optional(CONF_DOUBLE_BUFFER, default=False): cv.boolean,
            cv.Optional(CONF_ROTATION, default=0): cv.Range(min=0, max=3),
        }
    )
    .extend(cv.polling_component_schema("1s")),
    # .extend(spi.spi_device_schema()),
    cv.has_at_most_one_key(CONF_PAGES, CONF_LAMBDA),
)


async def to_code(config):


    addr_a = await cg.gpio_pin_expression(config[CONF_ADDRESS_A])
    addr_b = await cg.gpio_pin_expression(config[CONF_ADDRESS_B])
    addr_c = await cg.gpio_pin_expression(config[CONF_ADDRESS_C])
    addr_d = await cg.gpio_pin_expression(config[CONF_ADDRESS_D])

    clock = await cg.gpio_pin_expression(config[CONF_CLOCK_PIN])
    latch = await cg.gpio_pin_expression(config[CONF_LATCH_PIN])
    enable = await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])

    width = config[CONF_WIDTH]

    col_r1 = await cg.gpio_pin_expression(config[CONF_COLOR_R1])
    col_g1 = await cg.gpio_pin_expression(config[CONF_COLOR_G1])
    col_b1 = await cg.gpio_pin_expression(config[CONF_COLOR_B1])
    col_r2 = await cg.gpio_pin_expression(config[CONF_COLOR_R2])
    col_g2 = await cg.gpio_pin_expression(config[CONF_COLOR_G2])
    col_b2 = await cg.gpio_pin_expression(config[CONF_COLOR_B2])

    dual_buffer = config[CONF_DOUBLE_BUFFER]

    var = cg.new_Pvariable(config[CONF_ID])
    var = cg.new_Pvariable(
        config[CONF_ID],
        addr_a,
        addr_b,
        addr_c,
        addr_d,
        clock,
        latch,
        enable,
        width,
        [col_r1, col_g1, col_b1, col_r2, col_g2, col_b2],
        dual_buffer,
    )

    cg.add(var.begin())

    await display.register_display(var, config)
    # await spi.register_spi_device(var, config)

    cg.add(var.set_brightness(config[CONF_BRIGHTNESS]))



    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))

