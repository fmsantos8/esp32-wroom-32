#include "system/hal_system.h"
#include "hal_esp32_private.h"

#include "hal_memory.h"

hal_status_t hal_system_init(const hal_system_config_t *config) {
    hal_memory_info_t info;
    if (hal_memory_get_info(&info) == HAL_MEMORY_OK) {
        return HAL_STATUS_INVALID_STATE;
    }
    if (config == NULL) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }

    return hal_esp32_memory_status(hal_memory_init(config->memory_buffer, config->memory_capacity));
}

hal_status_t hal_system_seal(void) {
    return hal_esp32_memory_status(hal_memory_seal());
}
