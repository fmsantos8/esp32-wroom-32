#include "uart/hal_uart.h"
#include "hal_esp32_private.h"
#include "hal_memory.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_pm.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/** @brief Persistent UART state with a finite transmit-wait budget. */
struct hal_uart {
    uart_port_t port;                 /** SDK controller identifier. */
    hal_uart_data_width_t data_width; /** Payload validation rule. */
    TickType_t timeout;               /** Finite transmit budget in scheduler ticks. */
    bool initialized;                 /** True after complete construction. */
#if CONFIG_PM_ENABLE
    esp_pm_lock_handle_t clock_lock; /** Keeps APB stable, including pending bytes after timeout. */
#endif
};

static bool claimed_controllers[SOC_UART_NUM];

_Static_assert(sizeof(hal_uart_t) <= UINT32_MAX, "UART object exceeds arena size range");
_Static_assert(_Alignof(hal_uart_t) <= _Alignof(max_align_t), "UART object requires extended alignment");

static hal_status_t validate_config(const hal_uart_config_t *config, TickType_t *ticks) {
    if (config->instance >= SOC_UART_NUM || config->baudrate == 0 || config->timeout == 0 ||
        (config->data_width != HAL_UART_DATA_WIDTH_7 && config->data_width != HAL_UART_DATA_WIDTH_8) ||
        (config->parity != HAL_UART_PARITY_NONE && config->parity != HAL_UART_PARITY_EVEN && config->parity != HAL_UART_PARITY_ODD) ||
        (config->stop_count != HAL_UART_STOP_COUNT_1 && config->stop_count != HAL_UART_STOP_COUNT_2) ||
        (config->transmit_pin.port == config->receive_pin.port && config->transmit_pin.number == config->receive_pin.number)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    hal_status_t status = hal_esp32_pin_validate(config->transmit_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_validate(config->receive_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->transmit_pin.number) || config->baudrate < 80 || config->baudrate > UART_BITRATE_MAX) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    return hal_esp32_convert_timeout_to_ticks(config->timeout, ticks);
}

static hal_status_t configure_uart(hal_uart_t *object, const hal_uart_config_t *config) {
    gpio_config_t pin = {
        .pin_bit_mask = UINT64_C(1) << config->transmit_pin.number,
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // Establish idle-high TX and a digital RX before connecting the UART matrix.
    if (gpio_config(&pin) != ESP_OK || gpio_set_level((gpio_num_t)config->transmit_pin.number, 1) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }
    pin.pin_bit_mask = UINT64_C(1) << config->receive_pin.number;
    pin.mode = GPIO_MODE_INPUT;
    if (gpio_config(&pin) != ESP_OK) {
        return HAL_STATUS_ERROR;
    }

    esp_err_t error = ESP_OK;
    bool installed = false;
#if CONFIG_PM_ENABLE
    bool clock_acquired = false;
    error = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "hal_uart", &object->clock_lock);
    if (error == ESP_OK) {
        error = esp_pm_lock_acquire(object->clock_lock);
        clock_acquired = error == ESP_OK;
    }
#endif
    if (error == ESP_OK) {
        // TX buffering is disabled: only copied FIFO bytes count as accepted.
        error = uart_driver_install(object->port, UART_HW_FIFO_LEN(object->port) + 1, 0, 0, NULL, 0);
        installed = error == ESP_OK;
    }
    if (error == ESP_OK) {
        const uart_config_t uart_config = {
            .baud_rate = (int)config->baudrate,
            .data_bits = config->data_width == HAL_UART_DATA_WIDTH_7 ? UART_DATA_7_BITS : UART_DATA_8_BITS,
            .parity = config->parity == HAL_UART_PARITY_NONE ? UART_PARITY_DISABLE : config->parity == HAL_UART_PARITY_EVEN ? UART_PARITY_EVEN
                                                                                                                            : UART_PARITY_ODD,
            .stop_bits = config->stop_count == HAL_UART_STOP_COUNT_1 ? UART_STOP_BITS_1 : UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
        error = uart_param_config(object->port, &uart_config);
    }
    if (error == ESP_OK) {
        error = uart_set_mode(object->port, UART_MODE_UART);
    }
    if (error == ESP_OK) {
        error = uart_set_line_inverse(object->port, UART_SIGNAL_INV_DISABLE);
    }
    if (error == ESP_OK) {
        error = uart_set_pin(object->port, config->transmit_pin.number, config->receive_pin.number, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (error == ESP_OK) {
        // The TX completion wait enables its own interrupt when needed.
        error = uart_disable_intr_mask(object->port, UINT32_MAX);
    }
    if (error == ESP_OK) {
        if (gpio_hold_dis((gpio_num_t)config->transmit_pin.number) != ESP_OK ||
            gpio_hold_dis((gpio_num_t)config->receive_pin.number) != ESP_OK) {
            error = ESP_FAIL;
        }
    }
    if (error != ESP_OK) {
        if (installed) {
            uart_driver_delete(object->port);
        }
#if CONFIG_PM_ENABLE
        if (clock_acquired) {
            esp_pm_lock_release(object->clock_lock);
        }
        if (object->clock_lock != NULL) {
            esp_pm_lock_delete(object->clock_lock);
        }
#endif
        return error == ESP_ERR_NO_MEM ? HAL_STATUS_NO_MEMORY : HAL_STATUS_ERROR;
    }
    return HAL_STATUS_OK;
}

hal_status_t hal_uart_config(hal_uart_t **uart, const hal_uart_config_t *config) {
    if (uart == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    *uart = NULL;
    if (config == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    TickType_t ticks;
    hal_status_t status = validate_config(config, &ticks);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_claim(config->transmit_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    status = hal_esp32_pin_claim(config->receive_pin);
    if (status != HAL_STATUS_OK) {
        hal_esp32_pin_release(config->transmit_pin);
        return status;
    }
    bool busy = claimed_controllers[config->instance] || uart_is_driver_installed((uart_port_t)config->instance);
#if CONFIG_ESP_CONSOLE_UART
    busy = busy || config->instance == CONFIG_ESP_CONSOLE_UART_NUM;
#endif
    if (busy) {
        hal_esp32_pin_release(config->transmit_pin);
        hal_esp32_pin_release(config->receive_pin);
        return HAL_STATUS_RESOURCE_BUSY;
    }
    void *storage = NULL;
    status = hal_esp32_memory_status(hal_memory_alloc((uint32_t)sizeof(hal_uart_t), &storage));
    if (status != HAL_STATUS_OK) {
        hal_esp32_pin_release(config->transmit_pin);
        hal_esp32_pin_release(config->receive_pin);
        return status;
    }
    claimed_controllers[config->instance] = true;
    hal_uart_t *object = storage;
    *object = (hal_uart_t){.port = (uart_port_t)config->instance, .data_width = config->data_width, .timeout = ticks};
    status = configure_uart(object, config);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    object->initialized = true;
    *uart = object;
    return HAL_STATUS_OK;
}

hal_status_t hal_uart_write(hal_uart_t *uart, const uint8_t *buffer, uint32_t size, uint32_t *transmitted_count) {
    if (transmitted_count == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    *transmitted_count = 0;
    if (uart == NULL || (buffer == NULL && size != 0)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!uart->initialized) {
        return HAL_STATUS_NOT_INITIALIZED;
    }
    if (uart->data_width == HAL_UART_DATA_WIDTH_7) {
        for (uint32_t i = 0; i < size; ++i) {
            if (buffer[i] > 0x7F) {
                return HAL_STATUS_INVALID_ARGUMENT;
            }
        }
    }
    if (size == 0) {
        return HAL_STATUS_OK;
    }

    TickType_t start = xTaskGetTickCount();
    while (*transmitted_count < size) {
        if ((TickType_t)(xTaskGetTickCount() - start) >= uart->timeout) {
            return HAL_STATUS_TIMEOUT;
        }
        // Exclusive, sequential use keeps the SDK's private TX mutex uncontended.
        int accepted = uart_tx_chars(uart->port, (const char *)(buffer + *transmitted_count), size - *transmitted_count);
        if (accepted < 0 || (uint32_t)accepted > size - *transmitted_count) {
            return HAL_STATUS_ERROR;
        }
        *transmitted_count += (uint32_t)accepted;
        if (accepted == 0) {
            vTaskDelay(1);
        }
    }
    TickType_t elapsed = (TickType_t)(xTaskGetTickCount() - start);
    TickType_t remaining = elapsed < uart->timeout ? uart->timeout - elapsed : 0;
    esp_err_t error = uart_wait_tx_done(uart->port, remaining);
    if (error == ESP_OK) {
        return HAL_STATUS_OK;
    }
    return error == ESP_ERR_TIMEOUT ? HAL_STATUS_TIMEOUT : HAL_STATUS_ERROR;
}
