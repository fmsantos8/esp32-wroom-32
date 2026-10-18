#include "i2c/hal_i2c.h"
#include "hal_esp32_private.h"
#include "hal_memory.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

/** @brief Persistent synchronous bus and its reusable addressed endpoint. */
struct hal_i2c {
    i2c_master_bus_handle_t bus;    /** SDK controller handle. */
    i2c_master_dev_handle_t device; /** Endpoint whose address changes before each transfer. */
    int timeout;                    /** Finite SDK timeout in milliseconds. */
    bool initialized;               /** True after complete construction. */
};

static bool claimed_controllers[SOC_I2C_NUM];

_Static_assert(sizeof(hal_i2c_t) <= UINT32_MAX, "I2C object exceeds arena size range");
_Static_assert(_Alignof(hal_i2c_t) <= _Alignof(max_align_t), "I2C object requires extended alignment");

static hal_status_t validate_config(const hal_i2c_config_t *config) {
    if (config->instance >= SOC_I2C_NUM || config->frequency == 0 || config->timeout == 0 ||
        (config->clock_pin.port == config->data_pin.port && config->clock_pin.number == config->data_pin.number)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    hal_status_t status = hal_esp32_pin_validate(config->clock_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_validate(config->data_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->clock_pin.number) || !GPIO_IS_VALID_OUTPUT_GPIO(config->data_pin.number) ||
        config->frequency < 10000 || config->frequency > 400000) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    TickType_t ticks;
    return hal_esp32_convert_timeout_to_ticks(config->timeout, &ticks);
}

static hal_status_t transfer_status(esp_err_t status) {
    if (status == ESP_OK) {
        return HAL_STATUS_OK;
    }
    return status == ESP_ERR_TIMEOUT ? HAL_STATUS_TIMEOUT : HAL_STATUS_ERROR;
}

static hal_status_t configure_bus(hal_i2c_t *object, const hal_i2c_config_t *config) {
    const gpio_config_t pins = {
        .pin_bit_mask = (UINT64_C(1) << config->clock_pin.number) | (UINT64_C(1) << config->data_pin.number),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // Establish digital pads and high latches before the SDK enables open drain.
    if (gpio_config(&pins) != ESP_OK || gpio_set_level((gpio_num_t)config->clock_pin.number, 1) != ESP_OK ||
        gpio_set_level((gpio_num_t)config->data_pin.number, 1) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = config->instance,
        .scl_io_num = (gpio_num_t)config->clock_pin.number,
        .sda_io_num = (gpio_num_t)config->data_pin.number,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    esp_err_t error = i2c_new_master_bus(&bus_config, &object->bus);
    if (error == ESP_OK) {
        const i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x08,
            .scl_speed_hz = config->frequency,
        };
        // One synchronous endpoint serves every address without runtime allocation.
        error = i2c_master_bus_add_device(object->bus, &device_config, &object->device);
    }
    if (error == ESP_OK) {
        if (gpio_hold_dis((gpio_num_t)config->clock_pin.number) != ESP_OK ||
            gpio_hold_dis((gpio_num_t)config->data_pin.number) != ESP_OK) {
            error = ESP_FAIL;
        }
    }
    if (error != ESP_OK) {
        // SDK cleanup does not reopen arena storage or relinquish touched pins.
        if (object->device != NULL) {
            i2c_master_bus_rm_device(object->device);
        }
        if (object->bus != NULL) {
            i2c_del_master_bus(object->bus);
        }
        if (error == ESP_ERR_NO_MEM) {
            return HAL_STATUS_NO_MEMORY;
        }
        return error == ESP_ERR_NOT_FOUND || error == ESP_ERR_INVALID_STATE ? HAL_STATUS_RESOURCE_BUSY : HAL_STATUS_ERROR;
    }
    return HAL_STATUS_OK;
}

hal_status_t hal_i2c_config(hal_i2c_t **i2c, const hal_i2c_config_t *config) {
    if (i2c == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    *i2c = NULL;
    if (config == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    hal_status_t status = validate_config(config);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_claim(config->clock_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_claim(config->data_pin);
    if (status != HAL_STATUS_OK) {
        hal_esp32_pin_release(config->clock_pin);
        return status;
    }
    i2c_master_bus_handle_t existing = NULL;
    if (claimed_controllers[config->instance] || i2c_master_get_bus_handle(config->instance, &existing) == ESP_OK) {
        hal_esp32_pin_release(config->clock_pin);
        hal_esp32_pin_release(config->data_pin);
        return HAL_STATUS_RESOURCE_BUSY;
    }
    void *storage = NULL;
    status = hal_esp32_memory_status(hal_memory_alloc((uint32_t)sizeof(hal_i2c_t), &storage));
    if (status != HAL_STATUS_OK) {
        hal_esp32_pin_release(config->clock_pin);
        hal_esp32_pin_release(config->data_pin);
        return status;
    }
    claimed_controllers[config->instance] = true;
    hal_i2c_t *object = storage;
    *object = (hal_i2c_t){.timeout = (int)config->timeout};
    status = configure_bus(object, config);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    object->initialized = true;
    *i2c = object;
    return HAL_STATUS_OK;
}

static hal_status_t validate_transfer(hal_i2c_t *i2c, uint8_t address, const void *buffer, uint32_t size) {
    if (i2c == NULL || buffer == NULL || size == 0 || address < 0x08 || address > 0x77) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!i2c->initialized) {
        return HAL_STATUS_NOT_INITIALIZED;
    }
    return size > UINT16_MAX ? HAL_STATUS_NOT_SUPPORTED : HAL_STATUS_OK;
}

hal_status_t hal_i2c_write(hal_i2c_t *i2c, uint8_t address, const uint8_t *buffer, uint32_t size) {
    hal_status_t status = validate_transfer(i2c, address, buffer, size);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    esp_err_t error = i2c_master_device_change_address(i2c->device, address, i2c->timeout);
    if (error == ESP_OK) {
        error = i2c_master_transmit(i2c->device, buffer, size, i2c->timeout);
    }
    return transfer_status(error);
}

hal_status_t hal_i2c_read(hal_i2c_t *i2c, uint8_t address, uint8_t *buffer, uint32_t size) {
    hal_status_t status = validate_transfer(i2c, address, buffer, size);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    esp_err_t error = i2c_master_device_change_address(i2c->device, address, i2c->timeout);
    if (error == ESP_OK) {
        error = i2c_master_receive(i2c->device, buffer, size, i2c->timeout);
    }
    return transfer_status(error);
}
