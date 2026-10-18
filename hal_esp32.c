#include "hal_esp32_private.h"

#include "driver/gpio.h"
#include "esp_private/esp_gpio_reserve.h"

static uint64_t claimed_pins = 0;

hal_status_t hal_esp32_memory_status(hal_memory_status_t status) {
    switch (status) {
        case HAL_MEMORY_OK:
            return HAL_STATUS_OK;
        case HAL_MEMORY_INVALID_ARGUMENT:
            return HAL_STATUS_INVALID_ARGUMENT;
        case HAL_MEMORY_NOT_INITIALIZED:
            return HAL_STATUS_NOT_INITIALIZED;
        case HAL_MEMORY_INVALID_STATE:
            return HAL_STATUS_INVALID_STATE;
        case HAL_MEMORY_NO_MEMORY:
            return HAL_STATUS_NO_MEMORY;
        default:
            return HAL_STATUS_ERROR;
    }
}

hal_status_t hal_esp32_pin_validate(hal_pin_t pin) {
    if (pin.port != 0 || pin.number >= GPIO_NUM_MAX || pin.number == 20 || !GPIO_IS_VALID_GPIO(pin.number)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    // GPIO6..11 connect to module flash; GPIO37/38 are not exposed.
    if ((pin.number >= 6 && pin.number <= 11) || pin.number == 37 || pin.number == 38) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    return HAL_STATUS_OK;
}

hal_status_t hal_esp32_pin_claim(hal_pin_t pin) {
    hal_status_t status = hal_esp32_pin_validate(pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    hal_memory_info_t info;
    status = hal_esp32_memory_status(hal_memory_get_info(&info));
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (info.sealed) {
        return HAL_STATUS_INVALID_STATE;
    }

    uint64_t mask = UINT64_C(1) << pin.number;
    if ((claimed_pins & mask) != 0 || esp_gpio_is_reserved(mask)) {
        return HAL_STATUS_RESOURCE_BUSY;
    }
    claimed_pins |= mask;
    return HAL_STATUS_OK;
}

void hal_esp32_pin_release(hal_pin_t pin) {
    if (hal_esp32_pin_validate(pin) == HAL_STATUS_OK) {
        claimed_pins &= ~(UINT64_C(1) << pin.number);
    }
}
