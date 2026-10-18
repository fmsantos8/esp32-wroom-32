#include "system/hal_system.h"
#include "hal_esp32_private.h"

#include "hal_memory.h"

static hal_memory_t shared_memory;

static hal_status_t memory_status(hal_memory_status_t status) {
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

hal_status_t hal_system_init(const hal_system_config_t *config) {
    if (shared_memory.initialized) {
        return HAL_STATUS_INVALID_STATE;
    }
    if (config == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }

    /* ESP-IDF 6.1 initializes the clock tree and system/peripheral clocks in
     * cpu_start.c before app_main(). Use the SDK build configuration as-is;
     * no additional global hardware initialization is required here. */
    return memory_status(hal_memory_init(&shared_memory, config->memory_buffer, config->memory_capacity));
}

hal_status_t hal_system_seal(void) {
    return memory_status(hal_memory_seal(&shared_memory));
}

hal_status_t hal_esp32_memory_require_open(void) {
    hal_memory_info_t info;
    hal_status_t status = memory_status(hal_memory_get_info(&shared_memory, &info));
    if (status != HAL_STATUS_OK) {
        return status;
    }
    return info.sealed ? HAL_STATUS_INVALID_STATE : HAL_STATUS_OK;
}

hal_status_t hal_esp32_memory_alloc(uint32_t size, void **out_pointer) {
    return memory_status(hal_memory_alloc(&shared_memory, size, out_pointer));
}
