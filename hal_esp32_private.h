/**
 * @file hal_esp32_private.h
 * @author Fabricio Santos (fmsantos8@ucs.br)
 * @brief Internal services shared by this port's peripheral implementations.
 */
#ifndef HAL_ESP32_PRIVATE_H
#define HAL_ESP32_PRIVATE_H

#include <stdint.h>

#include "common/hal_pin.h"
#include "common/hal_status.h"

#include "hal_memory.h"

#include "freertos/FreeRTOS.h"

/**
 * @brief Translate the memory module's independent status into the HAL contract.
 *
 * @param[in] status The memory module's status.
 *
 * @return hal_status_t The corresponding HAL status.
 */
hal_status_t hal_esp32_memory_status(hal_memory_status_t status);

/**
 * @brief Validate a finite timeout in milliseconds and convert it to FreeRTOS ticks.
 *
 * @details Uses pdMS_TO_TICKS(), which rounds fractional ticks down.
 * Rejects values greater than INT_MAX for compatibility with the I2C driver,
 * values that overflow the conversion, and results equal to zero or
 * portMAX_DELAY. The conversion depends on configTICK_RATE_HZ.
 *
 * @param[in] timeout Timeout in milliseconds. Must be greater than zero.
 * @param[out] out_ticks Converted timeout. Modified only on success.
 *
 * @retval HAL_STATUS_OK Timeout validated and converted.
 * @retval HAL_STATUS_INVALID_ARGUMENT timeout is zero or out_ticks is NULL.
 * @retval HAL_STATUS_NOT_SUPPORTED Timeout exceeds the supported limits or
 * cannot produce a positive, finite tick count.
 */
hal_status_t hal_esp32_convert_timeout_to_ticks(uint32_t timeout, TickType_t *out_ticks);

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
