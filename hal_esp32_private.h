/**
 * @file hal_esp32_private.h
 * @brief Internal services shared by this port's peripheral implementations.
 */
#ifndef HAL_ESP32_PRIVATE_H
#define HAL_ESP32_PRIVATE_H

#include <stdint.h>

#include "common/hal_pin.h"
#include "common/hal_status.h"

#include "memory/hal_memory.h"

/**
 * @brief Translate the memory module's independent status into the HAL contract.
 *
 * @param[in] status The memory module's status.
 *
 * @return hal_status_t The corresponding HAL status.
 */
hal_status_t hal_esp32_memory_status(hal_memory_status_t status);

/**
 * @brief Validate bank zero, physical GPIO number and WROOM-32 module availability.
 *
 * @param pin[in] The pin to validate.
 *
 * @return hal_status_t HAL_STATUS_OK if the pin is valid, or an appropriate error code otherwise.
 */
hal_status_t hal_esp32_pin_validate(hal_pin_t pin);

/**
 * @brief Claim a pin for a HAL resource after checking the arena and SDK reservations.
 * Peripheral-specific capabilities must also be checked before claiming.
 * Construction and SDK pin configuration must be sequential.
 *
 * @param[in] pin The pin to claim.
 *
 * @return hal_status_t HAL_STATUS_OK if the pin was successfully claimed, or an appropriate error code otherwise.
 */
hal_status_t hal_esp32_pin_claim(hal_pin_t pin);

/**
 * @brief Release a previously claimed pin.
 *
 * @param[in] pin The pin to release.
 */
void hal_esp32_pin_release(hal_pin_t pin);

#endif
