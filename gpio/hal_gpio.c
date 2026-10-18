#include "gpio/hal_gpio.h"
#include "hal_esp32_private.h"
#include "hal_memory.h"

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"

struct hal_gpio {
    gpio_num_t pin;
    hal_gpio_direction_t direction;
    bool initialized;
};

_Static_assert(sizeof(hal_gpio_t) <= UINT32_MAX, "GPIO object exceeds arena size range");
_Static_assert(_Alignof(hal_gpio_t) <= _Alignof(max_align_t), "GPIO object requires extended alignment");

static hal_status_t validate_config(const hal_gpio_config_t *config) {
    if ((config->direction != HAL_GPIO_INPUT && config->direction != HAL_GPIO_OUTPUT) ||
        (config->pull != HAL_GPIO_PULL_NONE && config->pull != HAL_GPIO_PULL_UP && config->pull != HAL_GPIO_PULL_DOWN)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }

    hal_status_t status = hal_esp32_pin_validate(config->pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->pin.number) &&
        (config->direction == HAL_GPIO_OUTPUT || config->pull != HAL_GPIO_PULL_NONE)) {
        // ESP32 GPIO34..39 have neither output drivers nor internal pulls.
        return HAL_STATUS_NOT_SUPPORTED;
    }
    return HAL_STATUS_OK;
}

static hal_status_t configure_pin(const hal_gpio_config_t *config) {
    gpio_num_t pin = (gpio_num_t)config->pin.number;
    gpio_config_t sdk_config = {
        .pin_bit_mask = UINT64_C(1) << config->pin.number,
        .mode = config->direction == HAL_GPIO_INPUT ? GPIO_MODE_INPUT : GPIO_MODE_DISABLE,
        .pull_up_en = config->pull == HAL_GPIO_PULL_UP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = config->pull == HAL_GPIO_PULL_DOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // The SDK selects digital GPIO, disconnects peripheral output routing,
    // disables open drain/interrupts and establishes the requested pulls.
    if (gpio_config(&sdk_config) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }
    if (config->direction == HAL_GPIO_OUTPUT) {
        /* Load the output latch while its driver is disabled. */
        if (gpio_set_level(pin, config->initial_level) != ESP_OK) {
            return HAL_STATUS_ERROR;
        }
        sdk_config.mode = GPIO_MODE_OUTPUT;
        if (gpio_config(&sdk_config) != ESP_OK) {
            return HAL_STATUS_ERROR;
        }
    }

    // Apply the completed configuration even if the pad was held before reset.
    return gpio_hold_dis(pin) == ESP_OK ? HAL_STATUS_OK : HAL_STATUS_ERROR;
}

hal_status_t hal_gpio_config(hal_gpio_t **gpio, const hal_gpio_config_t *config) {
    if (gpio == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    *gpio = NULL;
    if (config == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }

    hal_status_t status = validate_config(config);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_claim(config->pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }

    void *storage = NULL;
    status = hal_esp32_memory_status(hal_memory_alloc((uint32_t)sizeof(hal_gpio_t), &storage));
    if (status != HAL_STATUS_OK) {
        hal_esp32_pin_release(config->pin);
        return status;
    }

    hal_gpio_t *object = storage;
    *object = (hal_gpio_t){
        .pin = (gpio_num_t)config->pin.number,
        .direction = config->direction,
        .initialized = false,
    };
    status = configure_pin(config);
    if (status != HAL_STATUS_OK) {
        // No arena rollback; retain this pin's claim after touching hardware.
        return status;
    }

    object->initialized = true;
    *gpio = object;
    return HAL_STATUS_OK;
}

hal_status_t hal_gpio_read(hal_gpio_t *gpio, bool *out_level) {
    if (gpio == NULL || out_level == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!gpio->initialized) {
        return HAL_STATUS_NOT_INITIALIZED;
    }
    if (gpio->direction != HAL_GPIO_INPUT) {
        return HAL_STATUS_NOT_SUPPORTED;
    }

    *out_level = gpio_get_level(gpio->pin) != 0;
    return HAL_STATUS_OK;
}

hal_status_t hal_gpio_write(hal_gpio_t *gpio, bool level) {
    if (gpio == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!gpio->initialized) {
        return HAL_STATUS_NOT_INITIALIZED;
    }
    if (gpio->direction != HAL_GPIO_OUTPUT) {
        return HAL_STATUS_NOT_SUPPORTED;
    }

    return gpio_set_level(gpio->pin, level) == ESP_OK ? HAL_STATUS_OK : HAL_STATUS_ERROR;
}
