#include "timer/hal_timer.h"
#include "hal_esp32_private.h"
#include "hal_memory.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_memory_utils.h"
#include "sdkconfig.h"

/** @brief Four logical identifiers backed by SDK-allocated dedicated timers. */
enum { TIMER_INSTANCE_COUNT = 4,
       TIMER_RESOLUTION_HZ = 1000000 };

/** @brief Persistent periodic hardware timer and its ISR callback. */
struct hal_timer {
    gptimer_handle_t device;       /** Dedicated SDK hardware timer. */
    hal_timer_callback_t callback; /** Application callback invoked from the timer ISR. */
    atomic_bool running;           /** Publishes application initialization to the ISR. */
    bool initialized;              /** True after complete construction, with counting stopped. */
};

static bool claimed_timers[TIMER_INSTANCE_COUNT];

_Static_assert(sizeof(hal_timer_t) <= UINT32_MAX, "Timer object exceeds arena size range");
_Static_assert(_Alignof(hal_timer_t) <= _Alignof(max_align_t), "Timer object requires extended alignment");
_Static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Timer ISR requires lock-free atomic bool");

static bool IRAM_ATTR on_alarm(gptimer_handle_t device, const gptimer_alarm_event_data_t *event, void *context) {
    (void)device;
    (void)event;
    hal_timer_t *timer = context;
    if (atomic_load_explicit(&timer->running, memory_order_acquire)) {
        timer->callback();
    }
    return false;
}

static hal_status_t configure_timer(hal_timer_t *object, uint32_t period) {
    const gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    esp_err_t error = gptimer_new_timer(&config, &object->device);
    if (error == ESP_OK) {
        uint32_t resolution = 0;
        error = gptimer_get_resolution(object->device, &resolution);
        if (error == ESP_OK && resolution != TIMER_RESOLUTION_HZ) {
            gptimer_del_timer(object->device);
            return HAL_STATUS_NOT_SUPPORTED;
        }
    }
    if (error == ESP_OK) {
        const gptimer_alarm_config_t alarm = {
            .alarm_count = (uint64_t)period * 1000,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        error = gptimer_set_alarm_action(object->device, &alarm);
    }
    if (error == ESP_OK) {
        const gptimer_event_callbacks_t callbacks = {.on_alarm = on_alarm};
        error = gptimer_register_event_callbacks(object->device, &callbacks, object);
    }
    if (error == ESP_OK) {
        error = gptimer_set_raw_count(object->device, 0);
    }
    if (error == ESP_OK) {
        // Enabling installs the interrupt and acquires the SDK clock lock now.
        error = gptimer_enable(object->device);
    }
    if (error != ESP_OK) {
        if (object->device != NULL) {
            gptimer_del_timer(object->device);
        }
        if (error == ESP_ERR_NO_MEM) {
            return HAL_STATUS_NO_MEMORY;
        }
        return error == ESP_ERR_NOT_FOUND ? HAL_STATUS_RESOURCE_BUSY : HAL_STATUS_ERROR;
    }
    return HAL_STATUS_OK;
}

hal_status_t hal_timer_config(hal_timer_t **timer, const hal_timer_config_t *config) {
    if (timer == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    *timer = NULL;
    if (config == NULL || config->callback == NULL || config->period == 0 || config->instance >= TIMER_INSTANCE_COUNT) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
#if CONFIG_GPTIMER_ISR_CACHE_SAFE
    if (!esp_ptr_in_iram(config->callback)) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
#endif
    hal_memory_info_t info;
    hal_status_t status = hal_esp32_memory_status(hal_memory_get_info(&info));
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (info.sealed) {
        return HAL_STATUS_INVALID_STATE;
    }
    if (claimed_timers[config->instance]) {
        return HAL_STATUS_RESOURCE_BUSY;
    }
    void *storage = NULL;
    status = hal_esp32_memory_status(hal_memory_alloc((uint32_t)sizeof(hal_timer_t), &storage));
    if (status != HAL_STATUS_OK) {
        return status;
    }
#if CONFIG_GPTIMER_ISR_CACHE_SAFE
    if (!esp_ptr_internal(storage)) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
#endif
    claimed_timers[config->instance] = true;
    hal_timer_t *object = storage;
    object->device = NULL;
    object->callback = config->callback;
    object->initialized = false;
    atomic_init(&object->running, false);
    status = configure_timer(object, config->period);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    object->initialized = true;
    *timer = object;
    return HAL_STATUS_OK;
}

hal_status_t hal_timer_start(hal_timer_t *timer) {
    if (timer == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!timer->initialized) {
        return HAL_STATUS_NOT_INITIALIZED;
    }
    if (atomic_load_explicit(&timer->running, memory_order_acquire)) {
        return HAL_STATUS_INVALID_STATE;
    }
    // Publish application state before the hardware may deliver its first alarm.
    atomic_store_explicit(&timer->running, true, memory_order_release);
    if (gptimer_start(timer->device) != ESP_OK) {
        atomic_store_explicit(&timer->running, false, memory_order_release);
        return HAL_STATUS_ERROR;
    }
    return HAL_STATUS_OK;
}
