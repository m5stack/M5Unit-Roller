/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __UNIT_ROLLERCAN_H
#define __UNIT_ROLLERCAN_H

#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "unit_roller_common.hpp"

/**
 * @def ROLLERCAN_MASTER
 * @brief Identifier for the RollerCAN master.
 */
#define ROLLERCAN_MASTER (0xA8)

/**
 * @def ROLLERCAN_ON
 * @brief Command to turn on the RollerCAN device.
 */
#define ROLLERCAN_ON (0x03)

/**
 * @def ROLLERCAN_OFF
 * @brief Command to turn off the RollerCAN device.
 */
#define ROLLERCAN_OFF (0x04)

/**
 * @def ROLLERCAN_CMD_ID
 * @brief Command identifier for RollerCAN commands.
 */
#define ROLLERCAN_CMD_ID (0x07)

/**
 * @def ROLLERCAN_UNPROTECT
 * @brief Command to unprotect the RollerCAN settings.
 */
#define ROLLERCAN_UNPROTECT (0x09)

/**
 * @def ROLLERCAN_FLASH
 * @brief Command to flash the RollerCAN firmware.
 */
#define ROLLERCAN_FLASH (0x0A)

/**
 * @def ROLLERCAN_BAUDRATE
 * @brief Command to set the baud rate for RollerCAN communication.
 */
#define ROLLERCAN_BAUDRATE (0x0B)

/**
 * @def ROLLERCAN_PROTECTION_ON
 * @brief Command to enable protection mode.
 */
#define ROLLERCAN_PROTECTION_ON (0x0C)

/**
 * @def ROLLERCAN_PROTECTION_OFF
 * @brief Command to disable protection mode.
 */
#define ROLLERCAN_PROTECTION_OFF (0x0D)

/**
 * @def ROLLERCAN_SAVE_FLASH
 * @brief Command to save settings to flash memory.
 */
#define ROLLERCAN_SAVE_FLASH (0x7002)

/**
 * @def ROLLERCAN_PROTECTION
 * @brief Command for accessing protection status.
 */
#define ROLLERCAN_PROTECTION (0x7003)

/**
 * @def ROLLERCAN_SWITCH
 * @brief Command to switch modes or states.
 */
#define ROLLERCAN_SWITCH (0x7004)

/**
 * @def ROLLERCAN_MODE
 * @brief Command to read or set operational mode.
 */
#define ROLLERCAN_MODE (0x7005)

/**
 * @def ROLLERCAN_SPEED
 * @brief Command to read or set speed parameters.
 */
#define ROLLERCAN_SPEED (0x700A)

/**
 * @def ROLLERCAN_SPEED_CURRENT
 * @brief Command to read or set current speed.
 */
#define ROLLERCAN_SPEED_CURRENT (0x7018)

/**
 * @def ROLLERCAN_SPEED_KP
 * @brief Command to read or set proportional gain for speed control.
 */
#define ROLLERCAN_SPEED_KP (0x7020)

/**
 * @def ROLLERCAN_SPEED_KI
 * @brief Command to read or set integral gain for speed control.
 */
#define ROLLERCAN_SPEED_KI (0x7021)

/**
 * @def ROLLERCAN_SPEED_KD
 * @brief Command to read or set derivative gain for speed control.
 */
#define ROLLERCAN_SPEED_KD (0x7022)

/**
 * @def ROLLERCAN_POSITION
 * @brief Command to read or set position parameters.
 */
#define ROLLERCAN_POSITION (0x7016)

/**
 * @def ROLLERCAN_POSITION_CURRENT
 * @brief Command to read or set current position.
 */
#define ROLLERCAN_POSITION_CURRENT (0x7017)

/**
 * @def ROLLERCAN_POSITION_KP
 * @brief Command to  read or set proportional gain for position control.
 */
#define ROLLERCAN_POSITION_KP (0x7023)

/**
 * @def ROLLERCAN_POSITION_KI
 * @brief Command to  read or set integral gain for position control.
 */
#define ROLLERCAN_POSITION_KI (0x7024)

/**
 * @def ROLLERCAN_POSITION_KD
 * @brief Command to  read or set derivative gain for position control.
 */
#define ROLLERCAN_POSITION_KD (0x7025)

/**
 * @def ROLLERCAN_CURRENT
 * @brief Command to  read or set current parameters.
 */
#define ROLLERCAN_CURRENT (0x7006)

/**
 * @def ROLLERCAN_ACTUAL_SPEED
 * @brief Command to read actual speed.
 */
#define ROLLERCAN_ACTUAL_SPEED (0x7030)

/**
 * @def ROLLERCAN_ACTUAL_POSITION
 * @brief Command to read actual position.
 */
#define ROLLERCAN_ACTUAL_POSITION (0x7031)

/**
 * @def ROLLERCAN_ACTUAL_CURRENT
 * @brief Command to read actual current.
 */
#define ROLLERCAN_ACTUAL_CURRENT (0x7032)

/**
 * @def ROLLERCAN_ENCODER
 * @brief Command to read or set encoder values.
 */
#define ROLLERCAN_ENCODER (0x7033)

/**
 * @def ROLLERCAN_VIN
 * @brief Command to read input voltage.
 */
#define ROLLERCAN_VIN (0x7034)

/**
 * @def ROLLERCAN_TEMP
 * @brief Command to read temperature.
 */
#define ROLLERCAN_TEMP (0x7035)

/**
 * @def ROLLERCAN_RGB_MODE
 * @brief Command to read or set RGB mode.
 */
#define ROLLERCAN_RGB_MODE (0x7050)

/**
 * @def ROLLERCAN_RGB_COLOR
 * @brief Command to read or set RGB color values.
 */
#define ROLLERCAN_RGB_COLOR (0x7051)

/**
 * @def ROLLERCAN_RGB_BRIGHTNESS
 * @brief Command to read or set the brightness of the RGB LED.
 */
#define ROLLERCAN_RGB_BRIGHTNESS (0x7052)

/**
 * @def ROLLERCAN_STATUS_CMD
 * @brief Command to request the status of the RollerCAN device.
 */
#define ROLLERCAN_STATUS_CMD (0x02)

/**
 * @def ROLLERCAN_READ_CMD
 * @brief Command to initiate a read operation from the RollerCAN device.
 */
#define ROLLERCAN_READ_CMD (0x11)

/**
 * @def ROLLERCAN_WRITE_CMD
 * @brief Command to initiate a write operation to the RollerCAN device.
 */
#define ROLLERCAN_WRITE_CMD (0x12)

/**
 * @def ROLLERCAN_READ_I2C_CMD
 * @brief Command to initiate an I2C read operation.
 */
#define ROLLERCAN_READ_I2C_CMD (0x13)

/**
 * @def ROLLERCAN_WRITE_I2C_CMD
 * @brief Command to initiate an I2C write operation.
 */
#define ROLLERCAN_WRITE_I2C_CMD (0x14)

/**
 * @def ROLLERCAN_READ_I2C_RAW_CMD
 * @brief Command to initiate a raw I2C read operation.
 */
#define ROLLERCAN_READ_I2C_RAW_CMD (0x15)

/**
 * @def ROLLERCAN_WRITE_I2C_RAW_CMD
 * @brief Command to initiate a raw I2C write operation.
 */
#define ROLLERCAN_WRITE_I2C_RAW_CMD (0x16)

class UnitRollerCAN {
public:
    /**
     * @brief Initializes the CAN bus with the specified baud rate.
     *
     * This function sets up the CAN bus by configuring the specified
     * baud rate and optionally assigning specific pins for RX and TX.
     * If the pin numbers are set to -1, the default hardware pins will be used.
     *
     * @param baud The desired baud rate for the CAN communication.
     *             Common values include 125000, 250000, 500000, etc.
     * @param rxPin (Optional) The pin number to be used for receiving
     *               CAN messages. Default is -1, indicating that the
     *               default RX pin should be used.
     * @param txPin (Optional) The pin number to be used for transmitting
     *               CAN messages. Default is -1, indicating that the
     *               default TX pin should be used.
     *
     * @note Make sure to configure the correct baud rate and pin assignments
     *       based on your specific hardware setup to ensure proper communication
     *       on the CAN bus.
     */
    void beginCAN(unsigned long baud, int8_t rxPin = -1, int8_t txPin = -1);

    /**
     * @brief Sends data over the CAN bus.
     *
     * This function constructs and transmits a CAN message with the specified
     * CAN ID, command ID, options, and data payload. It formats the message
     * according to the RollerCAN protocol and sends it out on the configured
     * CAN bus.
     *
     * @param can_id The identifier for the CAN message. This should be unique
     *               for each device or message type to ensure proper routing
     *               of messages on the CAN bus.
     * @param cmd_id The command identifier that specifies the action or request
     *               being made. This should correspond to the defined command
     *               IDs in the RollerCAN protocol.
     * @param option Additional options that may modify the behavior of the command.
     *               The meaning of these options depends on the specific command
     *               being sent.
     * @param data Pointer to the data payload to be sent with the CAN message.
     *             This is typically an array of bytes containing the information
     *             relevant to the command.
     *
     * @return true if the data was successfully sent; false otherwise.
     *
     * @note Ensure that the CAN bus is initialized and ready before calling
     *       this function. The function may fail if the bus is not properly
     *       configured or if there are communication issues.
     */
    bool sendData(uint8_t can_id, uint8_t cmd_id, uint16_t option, const uint8_t *data);

    /**
     * @brief Receives data from the CAN bus.
     *
     * This function waits for a CAN message to be received on the configured
     * bus. When a message is available, it processes the data and returns
     * the status of the operation. The function will typically handle
     * parsing the incoming message into its components (such as CAN ID,
     * command ID, options, and payload), making it ready for further use.
     *
     * @return A signed integer indicating the status of the received
     *         message:
     *         - A non-negative integer (e.g., 0 or greater) indicates that a
     *           message was successfully received and processed.
     *         - A negative integer indicates an error or that no message
     *           was available.
     *
     * @note This function may block until a message is received or may
     *       return immediately if implemented in a non-blocking manner.
     *       Ensure that the CAN bus is initialized and operational before
     *       calling this function.
     */
    int8_t receiveData();

    /**
     * @brief Retrieves a specific parameter from a motor.
     *
     * This function requests and returns the value of a specific parameter
     * associated with the motor identified by the given ID. The parameter is
     * identified by its unique parameter ID, allowing users to access various
     * attributes such as speed, position, or other motor-specific settings.
     *
     * @param id The identifier of the motor from which to retrieve the
     *           parameter. This should correspond to the specific motor's ID
     *           in the system.
     * @param parameterId The unique identifier for the parameter to be
     *                    retrieved. index For details, see Table 1
     *
     * @return The value of the requested parameter as a signed 32-bit integer.
     *         If the parameter retrieval fails or the specified motor ID or
     *         parameter ID is invalid, the function may return an error code
     *         (typically a negative value) to indicate failure.
     *
     * @note Ensure that the CAN bus is initialized and functioning properly
     *       before calling this function. Additionally, the function may
     *       involve communication delays depending on the response time of
     *       the motor controller.
     */
    int32_t getMotorParameter(uint8_t id, uint16_t parameterId);

    /**
     * @brief Sets a specific parameter for a motor.
     *
     * This function sends a command to update a specific parameter associated
     * with the motor identified by the given ID. The parameter is specified
     * by its unique parameter ID and modified using the provided parameter data.
     *
     * @param id The identifier of the motor for which the parameter is being set.
     *           This should correspond to the specific motor's ID in the system.
     * @param parameterId The unique identifier for the parameter to be updated.index For details, see Table 1
     *
     * @param parameterData The new value to set for the specified parameter.
     *                      This value is represented as a signed 32-bit integer.
     *
     * @return A signed integer indicating the result of the operation:
     *         - positive: indicates success in setting the parameter.
     *         - A negative value: indicates an error occurred (e.g., invalid motor
     *           ID, parameter ID, or communication failure).
     *
     * @note Ensure that the CAN bus is initialized and operational before
     *       calling this function. Be aware that the function may involve
     *       communication delays depending on the response time of the motor
     *       controller.
     */
    int8_t setMotorParameter(uint8_t id, uint16_t parameterId, int32_t parameterData);

    /**
     * @brief Sets the PID controller parameters for a motor.
     *
     * This function configures the PID (Proportional, Integral, Derivative)
     * controller settings for the motor identified by the given ID. The
     * specific parameter to be updated is specified by its unique parameter
     * ID, and the corresponding data for the parameter is provided.
     *
     * @param id The identifier of the motor for which the PID parameters are being set.
     *           This should correspond to the specific motor's ID in the system.
     * @param parameterId The unique identifier for the PID parameter to be updated.index For details, see Table 1
     *
     * @param parameterData The new value to set for the specified PID parameter.
     *                      This value is represented as an unsigned 32-bit integer
     *
     * @return A signed integer indicating the result of the operation:
     *         - positive: indicates success in setting the PID parameter.
     *         - A negative value: indicates an error occurred (e.g., invalid motor
     *           ID, parameter ID, or communication failure).
     */
    int8_t setMotorPID(uint8_t id, uint16_t parameterId, uint32_t parameterData);

    /**
     * @brief Reads a specific parameter from a motor via CAN over I2C communication.
     *
     * This function retrieves the value of a specific parameter associated
     * with the motor identified by the given ID through a CAN interface that
     * communicates with an I2C device. The parameter to be read is specified
     * by its unique parameter ID, and an address may be used to access
     * specific data locations on the I2C device.
     *
     * @param id The identifier of the motor from which to retrieve the
     *           parameter. This should correspond to the specific motor's ID
     *           in the system.
     * @param addr The address location within the I2C device's parameter set
     *             that specifies where to read the data from. This may be used
     *             to access different registers or sections of the I2C device's memory.
     * @param parameterId The unique identifier for the parameter to be
     *                    retrieved. index For details, see Table 1
     *
     * @return The value of the requested parameter as a signed 32-bit integer.
     *         If the parameter retrieval fails, or if the specified motor ID
     *         or parameter ID is invalid, the function may return an error code
     *         (typically a negative value) to indicate failure.
     *
     */
    int32_t readMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId);

    /**
     * @brief Writes a motor parameter to the specified motor.
     *
     * This function sends a command to the motor with the specified ID to write a
     * given parameter identified by the parameterId. The data for the parameter is
     * provided in the parameterData argument.
     *
     * @param id          The unique identifier of the motor to which the parameter
     *                    will be written.
     * @param addr        The address of the specific register or location within the
     *                    motor where the parameter will be written.
     * @param parameterId The identifier of the parameter to be set.index For details, see Table 1
     * @param parameterData The data value to be written to the specified parameter.
     *
     * @return Returns 1 on success, a negative error code on failure.
     */
    int8_t writeMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId, int32_t parameterData);

    /**
     * @brief Writes a PID parameter to the specified motor.
     *
     * This function sends a command to the motor with the specified ID to write a
     * given PID parameter identified by the parameterId. The data for the PID
     * parameter is provided in the parameterData argument.
     *
     * @param id          The unique identifier of the motor to which the PID
     *                    parameter will be written.
     * @param addr        The address of the specific register or location within the
     *                    motor where the PID parameter will be written.
     * @param parameterId The identifier of the PID parameter to be set.
     * @param parameterData The data value to be written to the specified PID
     *                      parameter.
     *
     * @return Returns 1 on success, a negative error code on failure.
     */
    int8_t writeMotorPID(uint8_t id, uint8_t addr, uint16_t parameterId, uint32_t parameterData);

    /**
     * @brief Reads a raw parameter from the specified motor.
     *
     * This function retrieves a raw parameter from the motor identified by the
     * specified ID. The parameter is accessed at the given address and register.
     *
     * @param id   The unique identifier of the motor from which the parameter will
     *             be read.
     * @param addr The address of the specific register or location within the motor
     *             from which the raw parameter will be read.
     * @param reg  The register number that specifies the particular parameter to
     *             read.
     *
     * @return Returns the raw parameter value on success, or a negative error code
     *         on failure.
     */
    int32_t readMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg);

    /**
     * @brief Writes a raw parameter to the specified motor.
     *
     * This function sends a command to the motor identified by the specified ID
     * to write a raw parameter at the given address and register. The parameter
     * data is provided along with additional options and length specifications.
     *
     * @param id           The unique identifier of the motor to which the raw
     *                     parameter will be written.
     * @param addr         The address of the specific location within the motor
     *                     where the raw parameter will be written.
     * @param reg          The register number that specifies the particular
     *                     parameter to write.
     * @param option       Additional options for writing the parameter (e.g.,
     *                     behavior settings).
     * @param len          The length of the data being written, must not exceed
     *                     7.
     * @param parameterData The data value to be written to the specified raw
     *                      parameter.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg, uint8_t option, uint8_t len,
                                  int32_t parameterData);

    /**
     * @brief Writes a raw PID parameter to the specified motor.
     *
     * This function sends a command to the motor identified by the specified ID
     * to write a raw PID parameter at the given address and register. The PID
     * parameter data is provided along with additional options and length
     * specifications.
     *
     * @param id           The unique identifier of the motor to which the raw PID
     *                     parameter will be written.
     * @param addr         The address of the specific location within the motor
     *                     where the raw PID parameter will be written.
     * @param reg          The register number that specifies the particular
     *                     PID parameter to write.
     * @param option       Additional options for writing the PID parameter (e.g.,
     *                     behavior settings).
     * @param len          The length of the data being written, must not exceed
     *                     7.
     * @param parameterData The data value to be written to the specified raw PID
     *                      parameter.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeMotorRawPID(uint8_t id, uint8_t addr, uint8_t reg, uint8_t option, uint8_t len, uint32_t parameterData);

    /**
     * @brief Enables or disables stall protection for the specified motor.
     *
     * This function sets the stall protection feature for the motor identified
     * by the specified ID.  When enabled, the motor will prevent damage due to
     * excessive load or stalling conditions.
     *
     * @param id  The unique identifier of the motor for which stall protection
     *            will be set.
     * @param en  A boolean value indicating whether to enable or disable stall
     *            protection:
     *            - true: Enable stall protection.
     *            - false: Disable stall protection.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setRemoveProtection(uint8_t id, bool en);

    /**
     * @brief Removes stall protection for the specified motor.
     *
     * This function disables the stall protection feature for the motor identified
     * by the specified ID.  Once unprotected, the motor is allowed to operate without
     * restrictions that prevent damage due to excessive load or stalling conditions.
     *
     * @param id  The unique identifier of the motor from which stall protection
     *            will be removed.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setUnprotect(uint8_t id);

    /**
     * @brief Saves the current parameters to flash memory for the specified motor.
     *
     * This function stores the current configuration and parameters of the motor
     * identified by the specified ID into its flash memory. This ensures that the
     * settings are retained even after power loss or reset.
     *
     * @param id  The unique identifier of the motor whose parameters will be saved to flash.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setSaveFlash(uint8_t id);

    /**
     * @brief Sets the baud rate for communication with the specified motor.
     *
     * This function configures the communication baud rate for the motor identified
     * by the specified ID. The baud rate defined in the `roller_bps_t` enumeration
     * determines the speed of data transmission over the CAN bus.
     *
     * @param id   The unique identifier of the motor for which the baud rate will be set.
     * @param bps  The desired baud rate, specified as a value from the
     *             `roller_bps_t` enumeration:
     *             - ROLLER_BPS_CAN_1000000: 1 Mbps
     *             - ROLLER_BPS_CAN_500000:  500 Kbps
     *             - ROLLER_BPS_CAN_125000:  125 Kbps
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setBPS(uint8_t id, roller_bps_t bps);

    /**
     * @brief Sets the output state for the specified motor.
     *
     * This function enables or disables the output of the motor identified by the
     * specified ID. When enabled, the motor can operate and respond to commands;
     * when disabled, the motor stops its operation and does not respond to commands.
     *
     * @param id  The unique identifier of the motor whose output state will be set.
     * @param en  A boolean value indicating whether to enable or disable the motor output:
     *            - true: Enable motor output (turn on).
     *            - false: Disable motor output (turn off).
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setOutput(uint8_t id, bool en);

    /**
     * @brief Sets the operating mode for the specified motor.
     *
     * This function configures the operating mode of the motor identified by the
     * specified ID. The mode determines how the motor will behave and respond to
     * commands. The available modes are defined in the `roller_mode_t` enumeration.
     *
     * @param id    The unique identifier of the motor whose mode will be set.
     * @param mode  The desired operating mode, specified as a value from the
     *              `roller_mode_t` enumeration:
     *              - ROLLER_MODE_SPEED:     Speed mode
     *              - ROLLER_MODE_POSITION:  Position mode
     *              - ROLLER_MODE_CURRENT:   Current mode
     *              - ROLLER_MODE_ENCODER:   Encoder mode
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setMode(uint8_t id, roller_mode_t mode);

    /**
     * @brief Sets a new identifier for the specified motor.
     *
     * This function allows you to change the unique identifier of the motor
     * identified by the specified ID. This is useful for reassigning motor IDs
     * in systems where multiple motors are used and need to be uniquely addressed.
     *
     * @param id      The current unique identifier of the motor to be renamed.
     *                Valid range: 0 to 255.
     * @param newId   The new unique identifier to assign to the motor.
     *                Valid range: 0 to 255.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setMotorID(uint8_t id, uint8_t newId);

    /**
     * @brief Sets the RGB mode for the specified motor.
     *
     * This function configures the RGB lighting mode of the motor identified by
     * the specified ID. The mode determines how the RGB lights will behave and can
     * be set to either a system default mode or a user-defined mode.
     *
     * @param id    The unique identifier of the motor whose RGB mode will be set.
     *              Valid range: 0 to 255.
     * @param mode  The desired RGB mode, specified as a value from the
     *              `roller_rgb_t` enumeration:
     *              - ROLLER_RGB_MODE_DEFAULT:      System default mode
     *              - ROLLER_RGB_MODE_USER_DEFINED:  User-defined mode
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setRGBMode(uint8_t id, roller_rgb_t mode);

    /**
     * @brief Sets the RGB color for the specified motor.
     *
     * This function configures the RGB color of the motor identified by the
     * specified ID. The color is represented as a 32-bit integer, typically
     * formatted in ARGB (Alpha, Red, Green, Blue) format.
     *
     * @param id      The unique identifier of the motor whose RGB color will be set.
     *                Valid range: 0 to 255.
     * @param color   The desired RGB color value, represented as a 32-bit integer.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setRGB(uint8_t id, int32_t color);

    /**
     * @brief Sets the RGB brightness for the specified motor.
     *
     * This function configures the brightness of the RGB lights on the motor
     * identified by the specified ID. The brightness can be set to a value
     * between 0 (off) and 100 (full brightness).
     *
     * @param id        The unique identifier of the motor whose RGB brightness
     *                  will be set. Valid range: 0 to 255.
     * @param brightness The desired brightness level, expressed as a percentage.
     *                  Valid range: 0 to 100. A value of 0 turns off the RGB lights,
     *                  while a value of 100 sets them to full brightness.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setRGBBrightness(uint8_t id, uint8_t brightness);

    /**
     * @brief Sets the speed for the specified motor.
     *
     * This function configures the speed of the motor identified by the
     * specified ID. The speed can be set to a positive value for clockwise
     * (CW) rotation, a negative value for counterclockwise (CCW) rotation,
     * or zero to stop the motor.
     *
     * @param id     The unique identifier of the motor whose speed will be set.
     *               Valid range: 0 to 255.
     * @param speed  The desired speed value.
     *               A positive value indicates clockwise rotation,
     *               a negative value indicates counterclockwise rotation,
     *               and zero stops the motor.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setSpeed(uint8_t id, int32_t speed);

    /**
     * @brief Sets the speed current for the specified motor.
     *
     * This function configures the speed current of the motor identified by
     * the specified ID.
     *
     * @param id           The unique identifier of the motor whose speed current
     *                     will be set. Valid range: 0 to 255.
     * @param speedCurrent The desired speed current value.
     *                     Valid range: -1200 to 1200.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setSpeedCurrent(uint8_t id, int32_t speedCurrent);

    /**
     * @brief Sets the speed proportional gain (Kp) for the specified motor.
     *
     * This function configures the speed proportional gain used in the speed
     * control loop for the motor identified by the specified ID. The Kp value
     * determines how aggressively the controller responds to the error in speed.
     *
     * @param id       The unique identifier of the motor whose speed Kp will be set.
     *                 Valid range: 0 to 255.
     * @param speedKp  The desired speed proportional gain value.
     *                 A higher value increases the responsiveness of the speed control,
     *                 while a lower value results in a more gradual response.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setSpeedKp(uint8_t id, uint32_t speedKp);

    /**
     * @brief Sets the speed integral gain (Ki) for the specified motor.
     *
     * This function configures the speed integral gain used in the speed
     * control loop for the motor identified by the specified ID. The Ki value
     * helps eliminate steady-state error in speed control by integrating the
     * error over time.
     *
     * @param id       The unique identifier of the motor whose speed Ki will be set.
     *                 Valid range: 0 to 255.
     * @param speedKi  The desired speed integral gain value.
     *                 A higher value increases the responsiveness to steady-state errors,
     *                 while a lower value results in slower integration and may lead to
     *                 longer settling times.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setSpeedKi(uint8_t id, uint32_t speedKi);

    /**
     * @brief Sets the speed integral gain (Kd) for the specified motor.
     *
     * This function configures the speed integral gain used in the speed
     * control loop for the motor identified by the specified ID. The Kd value
     * helps eliminate steady-state error in speed control by integrating the
     * error over time.
     *
     * @param id       The unique identifier of the motor whose speed Kd will be set.
     *                 Valid range: 0 to 255.
     * @param speedKd  The desired speed integral gain value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setSpeedKd(uint8_t id, uint32_t speedKd);

    /**
     * @brief Sets the target position for the specified motor.
     *
     * This function configures the desired position for the motor identified by
     * the specified ID. The motor will move toward this position as part of its
     * control loop.
     *
     * @param id       The unique identifier of the motor whose position will be set.
     *                 Valid range: 0 to 255.
     * @param position The desired target position value for the motor.
     *                 The interpretation of this value depends on the system's
     *                 coordinate definition (e.g., encoder counts, degrees).
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setPosition(uint8_t id, int32_t position);

    /**
     * @brief Sets the current position value for the specified motor.
     *
     * This function configures the current position of the motor identified by
     * the specified ID. The position current is typically used in control algorithms
     * to maintain or adjust the motor's position based on feedback.
     *
     * @param id                The unique identifier of the motor whose position
     *                          current will be set. Valid range: 0 to 255.
     * @param positionCurrent   The desired current position value for the motor.
     *                          Valid range: -1200 to 1200.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setPositionCurrent(uint8_t id, int32_t positionCurrent);

    /**
     * @brief Sets the position proportional gain (Kp) for the specified motor.
     *
     * This function configures the position proportional gain used in the position
     * control loop for the motor identified by the specified ID. The Kp value
     * determines how aggressively the controller responds to the error in position.
     *
     * @param id        The unique identifier of the motor whose position Kp will be set.
     *                  Valid range: 0 to 255.
     * @param positionKp The desired position proportional gain value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setPositionKp(uint8_t id, uint32_t positionKp);

    /**
     * @brief Sets the position integral gain (Ki) for the specified motor.
     *
     * This function configures the position integral gain used in the position
     * control loop for the motor identified by the specified ID. The Ki value
     * helps eliminate steady-state error in position control by integrating the
     * error over time.
     *
     * @param id        The unique identifier of the motor whose position Ki will be set.
     *                  Valid range: 0 to 255.
     * @param positionKi The desired position integral gain value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setPositionKi(uint8_t id, uint32_t positionKi);

    /**
     * @brief Sets the position derivative gain (Kd) for the specified motor.
     *
     * This function configures the position derivative gain used in the position
     * control loop for the motor identified by the specified ID. The Kd value
     * helps reduce overshoot and dampen the response of the system by reacting
     * to the rate of change of the error.
     *
     * @param id        The unique identifier of the motor whose position Kd will be set.
     *                  Valid range: 0 to 255.
     * @param positionKd The desired position derivative gain value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setPositionKd(uint8_t id, uint32_t positionKd);

    /**
     * @brief Sets the current for the specified motor.
     *
     * This function configures the current value to be applied to the motor
     * identified by the specified ID. The current value can be adjusted to control
     * the torque produced by the motor. The valid range for current is from -1200
     * to 1200.
     *
     * @param id      The unique identifier of the motor whose current will be set.
     *                Valid range: 0 to 255.
     * @param current The desired current value to be applied to the motor.
     *                Valid range: -1200 to 1200.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setCurrent(uint8_t id, int32_t current);

    /**
     * @brief Sets the encoder value for the specified motor.
     *
     * This function configures the encoder value for the motor identified by
     * the specified ID. This can be useful for initializing the encoder position
     * or for resetting it to a specific value in applications where precise control
     * of the motor's position is required.
     *
     * @param id      The unique identifier of the motor whose encoder value will be set.
     *                Valid range: 0 to 255.
     * @param encoder The desired encoder value to be applied to the motor.
     *                The range and meaning of this value depends on the specific
     *                implementation and resolution of the encoder.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t setEncoder(uint8_t id, int32_t encoder);

    /**
     * @brief Gets the current speed of the specified motor.
     *
     * This function retrieves the current speed value for the motor identified
     * by the specified ID. The speed is typically measured in units relevant to
     * the specific application (e.g., RPM).
     *
     * @param id      The unique identifier of the motor whose speed will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current speed of the motor, or a negative error code on failure.
     */
    int32_t getSpeed(uint8_t id);

    /**
     * @brief Gets the current speed control value for the specified motor.
     *
     * This function retrieves the current speed control value being applied to the
     * motor identified by the specified ID. This value reflects the output of the
     * speed controller.
     *
     * @param id      The unique identifier of the motor whose speed control value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current speed control value, or a negative error code on failure.
     */
    int32_t getSpeedCurrent(uint8_t id);

    /**
     * @brief Gets the proportional gain (Kp) value for speed control of the specified motor.
     *
     * This function retrieves the proportional gain used in the speed control loop
     * for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose speed Kp value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current speed proportional gain (Kp), or a negative error code on failure.
     */
    uint32_t getSpeedKp(uint8_t id);

    /**
     * @brief Gets the integral gain (Ki) value for speed control of the specified motor.
     *
     * This function retrieves the integral gain used in the speed control loop
     * for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose speed Ki value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current speed integral gain (Ki), or a negative error code on failure.
     */
    uint32_t getSpeedKi(uint8_t id);

    /**
     * @brief Gets the derivative gain (Kd) value for speed control of the specified motor.
     *
     * This function retrieves the derivative gain used in the speed control loop
     * for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose speed Kd value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current speed derivative gain (Kd), or a negative error code on failure.
     */
    uint32_t getSpeedKd(uint8_t id);

    /**
     * @brief Gets the current position of the specified motor.
     *
     * This function retrieves the current position value for the motor identified
     * by the specified ID. The position is typically measured in units relevant to
     * the specific application (e.g., encoder counts).
     *
     * @param id      The unique identifier of the motor whose position will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current position of the motor, or a negative error code on failure.
     */
    int32_t getPosition(uint8_t id);

    /**
     * @brief Gets the current position control value for the specified motor.
     *
     * This function retrieves the current position control value being applied to
     * the motor identified by the specified ID. This value reflects the output of
     * the position controller.
     *
     * @param id      The unique identifier of the motor whose position control value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current position control value, or a negative error code on failure.
     */
    int32_t getPositionCurrent(uint8_t id);

    /**
     * @brief Gets the proportional gain (Kp) value for position control of the specified motor.
     *
     * This function retrieves the proportional gain used in the position control loop
     * for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose position Kp value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current position proportional gain (Kp), or a negative error code on failure.
     */
    uint32_t getPositionKp(uint8_t id);

    /**
     * @brief Gets the integral gain (Ki) value for position control of the specified motor.
     *
     * This function retrieves the integral gain used in the position control loop
     * for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose position Ki value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current position integral gain (Ki), or a negative error code on failure.
     */
    uint32_t getPositionKi(uint8_t id);

    /**
     * @brief Gets the derivative gain (Kd) value for position control of the specified motor.
     *
     * This function retrieves the derivative gain used in the position control loop
     * for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose position Kd value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current position derivative gain (Kd), or a negative error code on failure.
     */
    uint32_t getPositionKd(uint8_t id);

    /**
     * @brief Gets the current value for the specified motor.
     *
     * This function retrieves the current being supplied to the motor identified by
     * the specified ID. This can be useful for monitoring motor performance and
     * diagnosing issues.
     *
     * @param id      The unique identifier of the motor whose current will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current value, or a negative error code on failure.
     */
    int32_t getCurrent(uint8_t id);

    /**
     * @brief Gets the encoder value for the specified motor.
     *
     * This function retrieves the current encoder reading for the motor identified
     * by the specified ID. This value indicates the position or movement of the motor.
     *
     * @param id      The unique identifier of the motor whose encoder value will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the encoder value, or a negative error code on failure.
     */
    int32_t getEncoder(uint8_t id);

    /**
     * @brief Gets the actual speed of the specified motor.
     *
     * This function retrieves the actual speed of the motor identified by the
     * specified ID, reflecting real-time performance.
     *
     * @param id      The unique identifier of the motor whose actual speed will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the actual speed of the motor, or a negative error code on failure.
     */
    int32_t getActualSpeed(uint8_t id);

    /**
     * @brief Gets the actual position of the specified motor.
     *
     * This function retrieves the actual position of the motor identified by the
     * specified ID, reflecting real-time feedback.
     *
     * @param id      The unique identifier of the motor whose actual position will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the actual position of the motor, or a negative error code on failure.
     */
    int32_t getActualPosition(uint8_t id);

    /**
     * @brief Gets the actual current being drawn by the specified motor.
     *
     * This function retrieves the actual current being consumed by the motor identified
     * by the specified ID, which can be valuable for performance assessment.
     *
     * @param id      The unique identifier of the motor whose actual current will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the actual current, or a negative error code on failure.
     */
    int32_t getActualCurrent(uint8_t id);

    /**
     * @brief Gets the actual input voltage for the specified motor.
     *
     * This function retrieves the actual input voltage (Vin) being supplied to
     * the motor identified by the specified ID. Monitoring this value can help in
     * assessing the power supply performance and ensuring it meets the motor's requirements.
     *
     * @param id      The unique identifier of the motor whose actual input voltage will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the actual input voltage, or a negative error code on failure.
     */
    int32_t getActualVin(uint8_t id);

    /**
     * @brief Gets the current temperature of the specified motor.
     *
     * This function retrieves the current temperature of the motor identified by
     * the specified ID. This information is critical for thermal management and
     * preventing overheating.
     *
     * @param id      The unique identifier of the motor whose temperature will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current temperature in degrees Celsius, or a negative error code on failure.
     */
    int32_t getActualTemp(uint8_t id);

    /**
     * @brief Gets the RGB mode setting for the specified motor.
     *
     * This function retrieves the current RGB mode configuration for the device
     * identified by the specified ID, if applicable.
     *
     * @param id      The unique identifier of the device whose RGB mode will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the RGB mode value, or a negative error code on failure.
     */
    int8_t getRGBMode(uint8_t id);

    /**
     * @brief Gets the RGB color values for the specified motor or device.
     *
     * This function retrieves the RGB color values currently set for the device
     * identified by the specified ID. The values are returned in the array provided.
     *
     * @param id          The unique identifier of the device whose RGB values will be retrieved.
     *                    Valid range: 0 to 255.
     * @param rgbValues   Pointer to an array of three uint8_t elements where the RGB values
     *                    (Red, Green, Blue) will be stored. It should have enough space allocated.
     *
     * @return Returns 0 on success, or a negative error code on failure.
     */
    int8_t getRGB(uint8_t id, uint8_t *rgbValues);

    /**
     * @brief Gets the brightness level of the RGB for the specified motor or device.
     *
     * This function retrieves the current brightness level of the RGB light for the
     * device identified by the specified ID.
     *
     * @param id      The unique identifier of the device whose RGB brightness will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current brightness level, or a negative error code on failure.
     */
    int8_t getRGBBrightness(uint8_t id);

    /**
     * @brief Gets the output state of the specified motor or device.
     *
     * This function retrieves the current output state (enabled/disabled) of the
     * motor or device identified by the specified ID.
     *
     * @param id      The unique identifier of the device whose output state will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the output state, or a negative error code on failure.
     */
    int8_t getOutput(uint8_t id);

    /**
     * @brief Gets the operation mode of the specified motor or device.
     *
     * This function retrieves the current operational mode of the motor or device
     * identified by the specified ID.
     *
     * @param id      The unique identifier of the device whose operation mode will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the current mode value, or a negative error code on failure.
     */
    int8_t getMode(uint8_t id);

    /**
     * @brief Gets the system status of the specified motor or device.
     *
     * This function retrieves the current system status of the motor or device
     * identified by the specified ID, which may include ready, faulted, etc.
     *
     * @param id      The unique identifier of the device whose system status will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the system status value, or a negative error code on failure.
     */
    int8_t getSysStatus(uint8_t id);

    /**
     * @brief Gets the last error code reported by the specified motor or device.
     *
     * This function retrieves the most recent error code generated by the motor
     * or device identified by the specified ID. The error code can provide valuable
     * information for diagnosing issues or malfunctions within the device.
     *
     * @param id      The unique identifier of the device whose error code will be retrieved.
     *                Valid range: 0 to 255.
     *
     * @return Returns the last error code, or a negative error code on failure.
     *         A return value of 0 typically indicates no error.
     */
    int8_t getError(uint8_t id);

    /**
     * @brief Enables or disables stall protection for the specified motor.
     *
     * This function allows the user to enable or disable the stall protection feature of
     * the motor identified by the specified ID. When enabled, the stall protection will prevent
     * the motor from overheating and potentially damaging itself during a stall condition.
     *
     * @param id      The unique identifier of the motor whose stall protection setting will be changed.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param en      A boolean value indicating whether to enable (true) or disable (false) the stall protection.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRemoveProtection(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Saves the parameters to flash memory for the specified motor.
     *
     * This function allows the user to save the current configuration or parameters of
     * the motor identified by the specified ID into the flash memory. This is useful for
     * preserving settings across power cycles.
     *
     * @param id      The unique identifier of the motor whose parameters will be saved.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param en      A boolean value indicating whether to execute the save operation (true).
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeSaveFlash(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Controls the output state for the specified motor.
     *
     * This function allows the user to enable or disable the output for the motor identified
     * by the specified ID. When enabled, the motor can respond to commands and produce motion.
     * When disabled, the motor will not operate even if it receives commands.
     *
     * @param id      The unique identifier of the motor whose output state will be controlled.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param en      A boolean value indicating whether to enable (true) or disable (false) the output.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeOutput(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Sets the operation mode for the specified motor.
     *
     * This function allows the user to set a specific mode of operation for the motor
     * identified by the specified ID. The mode is defined by the roller_mode_t enumeration.
     *
     * @param id      The unique identifier of the motor whose mode will be set.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param mode    The desired operation mode from the roller_mode_t enumeration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeMode(uint8_t id, uint8_t addr, roller_mode_t mode);

    /**
     * @brief Sets the speed for the specified motor.
     *
     * This function allows the user to set the desired speed for the motor identified
     * by the specified ID. The speed is given in appropriate units (e.g., rpm).
     *
     * @param id      The unique identifier of the motor whose speed will be set.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param speed   The desired speed value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeSpeed(uint8_t id, uint8_t addr, int32_t speed);

    /**
     * @brief Sets the current speed setting for the specified motor.
     *
     * This function updates the current speed setting for the motor identified by the specified ID.
     *
     * @param id          The unique identifier of the motor whose speed current will be set.
     *                    Valid range: 0 to 255.
     * @param addr        The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param speedCurrent The current speed setting value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeSpeedCurrent(uint8_t id, uint8_t addr, int32_t speedCurrent);

    /**
     * @brief Sets the proportional gain for the speed control loop.
     *
     * This function sets the proportional gain (Kp) for the speed control loop of the specified motor.
     *
     * @param id        The unique identifier of the motor whose speed Kp value will be set.
     *                  Valid range: 0 to 255.
     * @param addr      The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param speedKp  The proportional gain value for speed control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeSpeedKp(uint8_t id, uint8_t addr, uint32_t speedKp);

    /**
     * @brief Sets the integral gain for the speed control loop.
     *
     * This function sets the integral gain (Ki) for the speed control loop of the specified motor.
     *
     * @param id        The unique identifier of the motor whose speed Ki value will be set.
     *                  Valid range: 0 to 255.
     * @param addr      The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param speedKi   The integral gain value for speed control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeSpeedKi(uint8_t id, uint8_t addr, uint32_t speedKi);

    /**
     * @brief Sets the derivative gain for the speed control loop.
     *
     * This function sets the derivative gain (Kd) for the speed control loop of the specified motor.
     *
     * @param id        The unique identifier of the motor whose speed Kd value will be set.
     *                  Valid range: 0 to 255.
     * @param addr      The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param speedKd   The derivative gain value for speed control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeSpeedKd(uint8_t id, uint8_t addr, uint32_t speedKd);

    /**
     * @brief Sets the target position for the specified motor.
     *
     * This function allows the user to set a target position for the motor identified
     * by the specified ID. The position is given in appropriate units (e.g., encoder counts).
     *
     * @param id        The unique identifier of the motor whose position will be set.
     *                  Valid range: 0 to 255.
     * @param addr      The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param position  The desired target position value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writePosition(uint8_t id, uint8_t addr, int32_t position);

    /**
     * @brief Sets the current position setting for the specified motor.
     *
     * This function updates the current position setting for the motor identified by the specified ID.
     *
     * @param id             The unique identifier of the motor whose position current will be set.
     *                       Valid range: 0 to 255.
     * @param addr           The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param positionCurrent The current position setting value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writePositionCurrent(uint8_t id, uint8_t addr, int32_t positionCurrent);

    /**
     * @brief Sets the proportional gain for the position control loop.
     *
     * This function sets the proportional gain (Kp) for the position control loop of the specified motor.
     *
     * @param id            The unique identifier of the motor whose position Kp value will be set.
     *                      Valid range: 0 to 255.
     * @param addr          The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param positionKp    The proportional gain value for position control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writePositionKp(uint8_t id, uint8_t addr, uint32_t positionKp);

    /**
     * @brief Sets the integral gain for the position control loop.
     *
     * This function sets the integral gain (Ki) for the position control loop of the specified motor.
     *
     * @param id            The unique identifier of the motor whose position Ki value will be set.
     *                      Valid range: 0 to 255.
     * @param addr          The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param positionKi    The integral gain value for position control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writePositionKi(uint8_t id, uint8_t addr, uint32_t positionKi);

    /**
     * @brief Sets the derivative gain for the position control loop.
     *
     * This function sets the derivative gain (Kd) for the position control loop of the specified motor.
     *
     * @param id            The unique identifier of the motor whose position Kd value will be set.
     *                      Valid range: 0 to 255.
     * @param addr          The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param positionKd    The derivative gain value for position control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writePositionKd(uint8_t id, uint8_t addr, uint32_t positionKd);

    /**
     * @brief Sets the current limit for the specified motor.
     *
     * This function allows the user to set a maximum allowable current for the motor identified
     * by the specified ID. This is useful for protecting the motor from overheating or damage.
     *
     * @param id       The unique identifier of the motor whose current limit will be set.
     *                 Valid range: 0 to 255.
     * @param addr     The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param current  The maximum current limit value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeCurrent(uint8_t id, uint8_t addr, int32_t current);

    /**
     * @brief Sets the encoder value for the specified motor.
     *
     * This function allows the user to set the encoder value for the motor identified
     * by the specified ID. This can be used to reset the encoder or to provide a specific
     * starting value for angle or position.
     *
     * @param id       The unique identifier of the motor whose encoder value will be set.
     *                 Valid range: 0 to 255.
     * @param addr     The I2C address (0-127) of the slave device (motor controller) to communicate with.
     * @param encoder  The encoder value to set.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeEncoder(uint8_t id, uint8_t addr, int32_t encoder);

    /**
     * @brief Sets the RGB mode for the specified device.
     *
     * This function configures the RGB mode for the device identified by the specified ID.
     * The mode determines how the RGB LEDs behave (e.g., static color, breathing effect, etc.).
     *
     * @param id    The unique identifier of the device whose RGB mode will be set.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param mode  The desired RGB mode as defined in roller_rgb_t enumeration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRGBMode(uint8_t id, uint8_t addr, roller_rgb_t mode);

    /**
     * @brief Sets the brightness of the RGB LEDs for the specified device.
     *
     * This function sets the brightness level for the RGB LEDs on the device identified
     * by the specified ID.
     *
     * @param id        The unique identifier of the device whose RGB brightness will be set.
     *                  Valid range: 0 to 255.
     * @param addr      The I2C address (0-127) of the slave device to communicate with.
     * @param brightness The brightness level (0-255), where 0 is off and 255 is the maximum brightness.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRGBBrightness(uint8_t id, uint8_t addr, uint8_t brightness);

    /**
     * @brief Sets the RGB color for the specified device.
     *
     * This function allows the user to set the RGB color for the device identified
     * by the specified ID. The color is typically provided as an integer representing the
     * RGB value (e.g., 0xFF0000 for red).
     *
     * @param id     The unique identifier of the device whose RGB color will be set.
     *               Valid range: 0 to 255.
     * @param addr   The I2C address (0-127) of the slave device to communicate with.
     * @param color  The RGB color value to set.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRGB(uint8_t id, uint8_t addr, int32_t color);

    /**
     * @brief Reads the output status of the specified motor or device.
     *
     * This function retrieves the current output status of the motor or device identified
     * by the specified ID. The output could represent various states such as active, idle,
     * or fault conditions.
     *
     * @param id    The unique identifier of the device whose output status will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the output status on success, or a negative error code on failure.
     */
    int8_t readOutput(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current mode of the specified device.
     *
     * This function retrieves the current operational mode of the device identified by the specified ID.
     *
     * @param id    The unique identifier of the device whose mode will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current mode on success, or a negative error code on failure.
     */
    int8_t readMode(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current speed of the specified motor.
     *
     * This function retrieves the current speed of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose speed will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current speed on success, or a negative error code on failure.
     */
    int32_t readSpeed(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the proportional gain (Kp) for speed control of the specified motor.
     *
     * This function retrieves the proportional gain value used in the speed control algorithm
     * for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose Kp value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kp value on success, or a negative error code on failure.
     */
    uint32_t readSpeedKp(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the integral gain (Ki) for speed control of the specified motor.
     *
     * This function retrieves the integral gain value used in the speed control algorithm
     * for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose Ki value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Ki value on success, or a negative error code on failure.
     */
    uint32_t readSpeedKi(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the derivative gain (Kd) for speed control of the specified motor.
     *
     * This function retrieves the derivative gain value used in the speed control algorithm
     * for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose Kd value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kd value on success, or a negative error code on failure.
     */
    uint32_t readSpeedKd(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current position of the specified motor.
     *
     * This function retrieves the current position of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose position will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current position on success, or a negative error code on failure.
     */
    int32_t readPosition(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the proportional gain (Kp) for position control of the specified motor.
     *
     * This function retrieves the proportional gain value used in the position control algorithm
     * for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose Kp value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kp value on success, or a negative error code on failure.
     */
    uint32_t readPositionKp(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the integral gain (Ki) for position control of the specified motor.
     *
     * This function retrieves the integral gain value used in the position control algorithm
     * for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose Ki value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Ki value on success, or a negative error code on failure.
     */
    uint32_t readPositionKi(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the derivative gain (Kd) for position control of the specified motor.
     *
     * This function retrieves the derivative gain value used in the position control algorithm
     * for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose Kd value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kd value on success, or a negative error code on failure.
     */
    uint32_t readPositionKd(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current position of the specified motor.
     *
     * This function retrieves the current position of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose position current will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current position on success, or a negative error code on failure.
     */
    int32_t readPositionCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current speed of the specified motor.
     *
     * This function retrieves the current speed of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose speed current will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current speed on success, or a negative error code on failure.
     */
    int32_t readSpeedCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current consumption of the specified motor.
     *
     * This function retrieves the current consumption of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose current consumption will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current consumption on success, or a negative error code on failure.
     */
    int32_t readCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the encoder value of the specified motor.
     *
     * This function retrieves the encoder value for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose encoder value will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the encoder value on success, or a negative error code on failure.
     */
    int32_t readEncoder(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual speed of the specified motor.
     *
     * This function retrieves the actual speed of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose actual speed will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual speed on success, or a negative error code on failure.
     */
    int32_t readActualSpeed(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual position of the specified motor.
     *
     * This function retrieves the actual position of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose actual position will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual position on success, or a negative error code on failure.
     */
    int32_t readActualPosition(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual current consumption of the specified motor.
     *
     * This function retrieves the actual current consumption of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose actual current will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual current consumption on success, or a negative error code on failure.
     */
    int32_t readActualCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual input voltage of the specified motor.
     *
     * This function retrieves the actual input voltage supplied to the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose actual input voltage will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual input voltage on success, or a negative error code on failure.
     */
    int32_t readActualVin(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual temperature of the specified motor.
     *
     * This function retrieves the actual temperature of the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose actual temperature will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual temperature on success, or a negative error code on failure.
     */
    int32_t readActualTemp(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the RGB mode setting of the specified device.
     *
     * This function retrieves the current RGB mode setting of the device identified by the specified ID.
     *
     * @param id    The unique identifier of the device whose RGB mode will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the RGB mode on success, or a negative error code on failure.
     */
    int8_t readRGBMode(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the RGB color values of the specified device.
     *
     * This function retrieves the RGB color values from the device identified by the specified ID.
     *
     * @param id          The unique identifier of the device whose RGB values will be read.
     *                    Valid range: 0 to 255.
     * @param addr        The I2C address (0-127) of the slave device to communicate with.
     * @param rgbValues   Pointer to an array of at least three elements to store the RGB values.
     *                    The first element will hold the red value, the second the green value,
     *                    and the third the blue value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t readRGB(uint8_t id, uint8_t addr, uint8_t *rgbValues);

    /**
     * @brief Reads the brightness level of the RGB LED of the specified device.
     *
     * This function retrieves the current brightness level of the RGB LED for the device identified by the specified
     * ID.
     *
     * @param id    The unique identifier of the device whose RGB brightness will be read.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the brightness level on success, or a negative error code on failure.
     */
    int8_t readRGBBrightness(uint8_t id, uint8_t addr);

    /**
     * @brief Enables or disables the stall protection feature for the specified motor.
     *
     * This function allows the user to enable or disable stall protection for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose stall protection setting will be changed.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param en    A boolean value indicating whether to enable (true) or disable (false) stall protection.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeResetStallProtect(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Enables or disables the position range protection feature for the specified motor.
     *
     * This function allows the user to enable or disable position range protection for the motor identified by the
     * specified ID.
     *
     * @param id    The unique identifier of the motor whose position range protection setting will be changed.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param en    A boolean value indicating whether to enable (true) or disable (false) position range protection.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writePosRangeProtect(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Enables or disables raw output mode for the specified motor.
     *
     * This function allows the user to enable or disable raw output mode for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose raw output setting will be changed.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param en    A boolean value indicating whether to enable (true) or disable (false) raw output mode.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawOutput(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Sets the mode for the raw control of the specified motor.
     *
     * This function allows the user to set the control mode for raw operation of the motor identified by the specified
     * ID.
     *
     * @param id    The unique identifier of the motor whose control mode will be set.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param mode  The desired mode for raw control operation as defined by the roller_mode_t enumeration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawMode(uint8_t id, uint8_t addr, roller_mode_t mode);

    /**
     * @brief Sets the raw speed for the specified motor.
     *
     * This function allows the user to set the desired raw speed for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose speed will be set.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param speed The desired raw speed value. The unit of measurement depends on the system configuration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawSpeed(uint8_t id, uint8_t addr, int32_t speed);

    /**
     * @brief Sets the raw speed current for the specified motor.
     *
     * This function allows the user to set the desired current for the raw speed control of the motor identified by the
     * specified ID.
     *
     * @param id            The unique identifier of the motor whose speed current will be set.
     *                      Valid range: 0 to 255.
     * @param addr          The I2C address (0-127) of the slave device to communicate with.
     * @param speedCurrent  The desired current value corresponding to the speed setting.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawSpeedCurrent(uint8_t id, uint8_t addr, int32_t speedCurrent);

    /**
     * @brief Sets the proportional gain for speed control of the specified motor.
     *
     * This function configures the proportional gain (Kp) used in the speed control algorithm for the motor identified
     * by the specified ID.
     *
     * @param id       The unique identifier of the motor whose Kp value will be set.
     *                 Valid range: 0 to 255.
     * @param addr     The I2C address (0-127) of the slave device to communicate with.
     * @param speedKp  The proportional gain value for speed control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawSpeedKp(uint8_t id, uint8_t addr, uint32_t speedKp);

    /**
     * @brief Sets the integral gain for speed control of the specified motor.
     *
     * This function configures the integral gain (Ki) used in the speed control algorithm for the motor identified by
     * the specified ID.
     *
     * @param id       The unique identifier of the motor whose Ki value will be set.
     *                 Valid range: 0 to 255.
     * @param addr     The I2C address (0-127) of the slave device to communicate with.
     * @param speedKi  The integral gain value for speed control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawSpeedKi(uint8_t id, uint8_t addr, uint32_t speedKi);

    /**
     * @brief Sets the derivative gain for speed control of the specified motor.
     *
     * This function configures the derivative gain (Kd) used in the speed control algorithm for the motor identified by
     * the specified ID.
     *
     * @param id       The unique identifier of the motor whose Kd value will be set.
     *                 Valid range: 0 to 255.
     * @param addr     The I2C address (0-127) of the slave device to communicate with.
     * @param speedKd  The derivative gain value for speed control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawSpeedKd(uint8_t id, uint8_t addr, uint32_t speedKd);

    /**
     * @brief Sets the raw position for the specified motor.
     *
     * This function allows the user to set the desired raw position for the motor identified by the specified ID.
     *
     * @param id        The unique identifier of the motor whose position will be set.
     *                  Valid range: 0 to 255.
     * @param addr      The I2C address (0-127) of the slave device to communicate with.
     * @param position  The desired raw position value. The unit of measurement depends on the system configuration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawPosition(uint8_t id, uint8_t addr, int32_t position);

    /**
     * @brief Sets the raw position current for the specified motor.
     *
     * This function allows the user to set the desired current for the raw position control of the motor identified by
     * the specified ID.
     *
     * @param id                The unique identifier of the motor whose position current will be set.
     *                         Valid range: 0 to 255.
     * @param addr              The I2C address (0-127) of the slave device to communicate with.
     * @param positionCurrent   The desired current value corresponding to the position setting.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawPositionCurrent(uint8_t id, uint8_t addr, int32_t positionCurrent);

    /**
     * @brief Sets the proportional gain for position control of the specified motor.
     *
     * This function configures the proportional gain (Kp) used in the position control algorithm for the motor
     * identified by the specified ID.
     *
     * @param id             The unique identifier of the motor whose Kp value will be set.
     *                       Valid range: 0 to 255.
     * @param addr           The I2C address (0-127) of the slave device to communicate with.
     * @param positionKp    The proportional gain value for position control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawPositionKp(uint8_t id, uint8_t addr, uint32_t positionKp);

    /**
     * @brief Sets the integral gain for position control of the specified motor.
     *
     * This function configures the integral gain (Ki) used in the position control algorithm for the motor identified
     * by the specified ID.
     *
     * @param id             The unique identifier of the motor whose Ki value will be set.
     *                       Valid range: 0 to 255.
     * @param addr           The I2C address (0-127) of the slave device to communicate with.
     * @param positionKi     The integral gain value for position control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawPositionKi(uint8_t id, uint8_t addr, uint32_t positionKi);

    /**
     * @brief Sets the derivative gain for position control of the specified motor.
     *
     * This function configures the derivative gain (Kd) used in the position control algorithm for the motor identified
     * by the specified ID.
     *
     * @param id             The unique identifier of the motor whose Kd value will be set.
     *                       Valid range: 0 to 255.
     * @param addr           The I2C address (0-127) of the slave device to communicate with.
     * @param positionKd     The derivative gain value for position control.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawPositionKd(uint8_t id, uint8_t addr, uint32_t positionKd);

    /**
     * @brief Sets the raw current for the specified motor.
     *
     * This function allows the user to set the desired raw current for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose current will be set.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device to communicate with.
     * @param current The desired raw current value. The unit of measurement depends on the system configuration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawCurrent(uint8_t id, uint8_t addr, int32_t current);

    /**
     * @brief Sets the raw encoder value for the specified motor.
     *
     * This function allows the user to set the desired raw encoder value for the motor identified by the specified ID.
     *
     * @param id      The unique identifier of the motor whose encoder value will be set.
     *                Valid range: 0 to 255.
     * @param addr    The I2C address (0-127) of the slave device to communicate with.
     * @param encoder The desired raw encoder value.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawEncoder(uint8_t id, uint8_t addr, int32_t encoder);

    /**
     * @brief Enables or disables the flash mode for the specified motor.
     *
     * This function allows the user to enable or disable the flash feature for the motor identified by the specified
     * ID.
     *
     * @param id   The unique identifier of the motor whose flash mode will be set.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     * @param en   True to enable flash mode, false to disable it.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawFlash(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Sets the RGB mode for the specified motor.
     *
     * This function configures the RGB mode for the motor identified by the specified ID.
     *
     * @param id    The unique identifier of the motor whose RGB mode will be set.
     *              Valid range: 0 to 255.
     * @param addr  The I2C address (0-127) of the slave device to communicate with.
     * @param mode  The desired RGB mode as defined by roller_rgb_t enumeration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawRGBMode(uint8_t id, uint8_t addr, roller_rgb_t mode);

    /**
     * @brief Sets the RGB color for the specified motor.
     *
     * This function allows the user to set the desired RGB color for the motor identified by the specified ID.
     *
     * @param id     The unique identifier of the motor whose RGB color will be set.
     *               Valid range: 0 to 255.
     * @param addr   The I2C address (0-127) of the slave device to communicate with.
     * @param color  The desired RGB color value. The format may depend on the system configuration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawRGB(uint8_t id, uint8_t addr, int32_t color);

    /**
     * @brief Sets the brightness for the RGB LED of the specified motor.
     *
     * This function allows the user to set the desired brightness level for the RGB LED
     * of the motor identified by the specified ID. Brightness is typically represented
     * as a value from 0 (off) to 255 (full brightness).
     *
     * @param id         The unique identifier of the motor whose RGB brightness will be set.
     *                   Valid range: 0 to 255.
     * @param addr       The I2C address (0-127) of the slave device to communicate with.
     * @param brightness  The desired brightness level for the RGB LED (0-255).
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawRGBBrightness(uint8_t id, uint8_t addr, uint8_t brightness);

    /**
     * @brief Sets a new I2C address for the specified device.
     *
     * This function allows the user to change the I2C address of the motor identified
     * by the specified ID. This is useful when multiple devices are connected on the
     * same I2C bus and need to have unique addresses.
     *
     * @param id        The unique identifier of the motor whose I2C address will be changed.
     *                  Valid range: 0 to 255.
     * @param addr      The current I2C address (0-127) of the slave device.
     * @param newAddr   The new I2C address to assign to the motor (0-127).
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeRawI2CAddress(uint8_t id, uint8_t addr, uint8_t newAddr);

    /**
     * @brief Enables or disables stall protection for the specified motor.
     *
     * This function allows the user to enable or disable stall protection for the motor
     * identified by the specified ID. When enabled, it helps to prevent damage due to
     * excessive load or stalling.
     *
     * @param id   The unique identifier of the motor whose stall protection will be set.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     * @param en   True to enable stall protection, false to disable it.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeStallProtect(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Enables or disables a button for the specified motor.
     *
     * This function allows the user to enable or disable the button functionality for
     * the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose button functionality will be set.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     * @param en   True to enable the button functionality, false to disable it.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeButton(uint8_t id, uint8_t addr, bool en);

    /**
     * @brief Sets the motor ID for the specified device.
     *
     * This function allows the user to set a new motor ID for the specified device,
     * which is useful for identifying different motors in a system.
     *
     * @param id       The unique identifier of the motor whose ID will be set.
     *                 Valid range: 0 to 255.
     * @param addr     The I2C address (0-127) of the slave device to communicate with.
     * @param motorId  The new motor ID to assign to the device (0-255).
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeMotorID(uint8_t id, uint8_t addr, uint8_t motorId);

    /**
     * @brief Sets the BPS (Beats Per Second) setting for the specified motor.
     *
     * This function allows the user to configure the beats per second (BPS)
     * setting for the motor identified by the specified ID. This is typically used
     * in applications involving timed operations or rhythmic functions.
     *
     * @param id   The unique identifier of the motor whose BPS setting will be configured.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     * @param bps  The desired beats per second setting as defined by roller_bps_t enumeration.
     *
     * @return Returns 1 on success, or a negative error code on failure.
     */
    int8_t writeBPS(uint8_t id, uint8_t addr, roller_bps_t bps);

    /**
     * @brief Reads the raw output value of the specified motor.
     *
     * This function retrieves the current raw output value for the motor identified
     * by the specified ID. The output value represents the motor's control signal.
     *
     * @param id   The unique identifier of the motor whose output will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the raw output value on success, or a negative error code on failure.
     */
    int8_t readRawOutput(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current mode of the specified motor.
     *
     * This function retrieves the current operating mode of the motor identified
     * by the specified ID. The mode indicates how the motor is currently configured
     * to operate (e.g., speed control, position control, etc.).
     *
     * @param id   The unique identifier of the motor whose mode will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current mode value on success, or a negative error code on failure.
     */
    int8_t readRawMode(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current speed of the specified motor.
     *
     * This function retrieves the current speed of the motor identified by the
     * specified ID. The speed is typically expressed in units relevant to the application
     * (e.g., RPM).
     *
     * @param id   The unique identifier of the motor whose speed will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current speed value on success, or a negative error code on failure.
     */
    int32_t readRawSpeed(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current speed current of the specified motor.
     *
     * This function retrieves the current drawn by the motor while it is operating at its
     * current speed. This is useful for monitoring the motor's power consumption and
     * performance.
     *
     * @param id   The unique identifier of the motor whose speed current will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current speed current value on success, or a negative error code on failure.
     */
    int32_t readRawSpeedCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the proportional gain (Kp) for speed control of the specified motor.
     *
     * This function retrieves the Kp (proportional gain) value used in the PID control
     * algorithm for speed regulation of the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose Kp value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kp value on success, or a negative error code on failure.
     */
    uint32_t readRawSpeedKp(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the integral gain (Ki) for speed control of the specified motor.
     *
     * This function retrieves the Ki (integral gain) value used in the PID control
     * algorithm for speed regulation of the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose Ki value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Ki value on success, or a negative error code on failure.
     */
    uint32_t readRawSpeedKi(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the derivative gain (Kd) for speed control of the specified motor.
     *
     * This function retrieves the Kd (derivative gain) value used in the PID control
     * algorithm for speed regulation of the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose Kd value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kd value on success, or a negative error code on failure.
     */
    uint32_t readRawSpeedKd(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current position of the specified motor.
     *
     * This function retrieves the current position of the motor identified by the
     * specified ID. The position is expressed in units relevant to the application
     * (e.g., encoder counts).
     *
     * @param id   The unique identifier of the motor whose position will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current position value on success, or a negative error code on failure.
     */
    int32_t readRawPosition(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current position current of the specified motor.
     *
     * This function retrieves the current drawn by the motor while it is operating at its
     * current position. This is useful for monitoring the motor's power consumption and
     * performance.
     *
     * @param id   The unique identifier of the motor whose position current will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current position current value on success, or a negative error code on failure.
     */
    int32_t readRawPositionCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the proportional gain (Kp) for position control of the specified motor.
     *
     * This function retrieves the Kp (proportional gain) value used in the PID control
     * algorithm for position regulation of the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose Kp value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kp value on success, or a negative error code on failure.
     */
    uint32_t readRawPositionKp(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the integral gain (Ki) for position control of the specified motor.
     *
     * This function retrieves the Ki (integral gain) value used in the PID control
     * algorithm for position regulation of the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose Ki value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Ki value on success, or a negative error code on failure.
     */
    uint32_t readRawPositionKi(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the derivative gain (Kd) for position control of the specified motor.
     *
     * This function retrieves the Kd (derivative gain) value used in the PID control
     * algorithm for position regulation of the motor identified by the specified ID.
     *
     * @param id   The unique identifier of the motor whose Kd value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the Kd value on success, or a negative error code on failure.
     */
    uint32_t readRawPositionKd(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the current drawn by the specified motor.
     *
     * This function retrieves the electrical current consumed by the motor identified
     * by the specified ID. This is useful for monitoring the motor's power consumption.
     *
     * @param id   The unique identifier of the motor whose current will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the current value on success, or a negative error code on failure.
     */
    int32_t readRawCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the encoder value for the specified motor.
     *
     * This function retrieves the raw encoder value from the motor identified by the
     * specified ID. The encoder value represents the position of the motor in terms of
     * counts or ticks.
     *
     * @param id   The unique identifier of the motor whose encoder value will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the encoder value on success, or a negative error code on failure.
     */
    int32_t readRawEncoder(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual speed of the specified motor.
     *
     * This function retrieves the current speed of the motor identified by the specified
     * ID. The speed is typically expressed in units such as RPM or counts per second.
     *
     * @param id   The unique identifier of the motor whose actual speed will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual speed value on success, or a negative error code on failure.
     */
    int32_t readRawActualSpeed(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual position of the specified motor.
     *
     * This function retrieves the current position of the motor identified by the
     * specified ID. The position is expressed in units relevant to the application
     * (e.g., encoder counts).
     *
     * @param id   The unique identifier of the motor whose actual position will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual position value on success, or a negative error code on failure.
     */
    int32_t readRawActualPosition(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual current consumed by the specified motor.
     *
     * This function retrieves the current being drawn by the motor identified by the
     * specified ID during its operation. This information is crucial for performance
     * analysis and diagnostics.
     *
     * @param id   The unique identifier of the motor whose actual current will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual current value on success, or a negative error code on failure.
     */
    int32_t readRawActualCurrent(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual input voltage of the specified motor controller.
     *
     * This function retrieves the actual input voltage (Vin) being supplied to the
     * motor identified by the specified ID. This information can help diagnose power
     * supply issues.
     *
     * @param id   The unique identifier of the motor whose actual input voltage will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual input voltage value on success, or a negative error code on failure.
     */
    int32_t readRawActualVin(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the actual temperature of the specified motor.
     *
     * This function retrieves the current temperature of the motor identified by the
     * specified ID. Monitoring the temperature is essential for preventing thermal
     * overload and ensuring safe operation.
     *
     * @param id   The unique identifier of the motor whose actual temperature will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the slave device to communicate with.
     *
     * @return Returns the actual temperature value on success, or a negative error code on failure.
     */
    int32_t readRawActualTemp(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the RGB mode of the specified device.
     *
     * This function retrieves the current RGB mode setting for the device identified
     * by the specified ID. Different modes may correspond to different LED behaviors.
     *
     * @param id   The unique identifier of the device whose RGB mode will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the RGB mode value on success, or a negative error code on failure.
     */
    int8_t readRawRGBMode(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the RGB values of the specified device.
     *
     * This function retrieves the current RGB color values (Red, Green, Blue) from the
     * device identified by the specified ID. The RGB values are stored in the provided
     * array.
     *
     * @param id          The unique identifier of the device whose RGB values will be read.
     *                    Valid range: 0 to 255.
     * @param addr        The I2C address (0-127) of the device to communicate with.
     * @param rgbValues   Pointer to an array where the RGB values will be stored.
     *                    The array must have at least three elements to store R, G, B.
     *
     * @return Returns 0 on success, or a negative error code on failure.
     */
    int8_t readRawRGB(uint8_t id, uint8_t addr, uint8_t *rgbValues);

    /**
     * @brief Reads the brightness level of the RGB LEDs of the specified device.
     *
     * This function retrieves the current brightness setting for the RGB LEDs of the
     * device identified by the specified ID.
     *
     * @param id   The unique identifier of the device whose RGB brightness will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the brightness level on success, or a negative error code on failure.
     */
    int8_t readRawRGBBrightness(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the firmware version of the specified device.
     *
     * This function retrieves the current firmware version of the device identified by
     * the specified ID. This can be useful for compatibility checks and troubleshooting.
     *
     * @param id   The unique identifier of the device whose firmware version will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the firmware version number on success, or a negative error code on failure.
     */
    int8_t readRawFirmwareVersion(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the firmware version of the specified device.
     *
     * This function retrieves the current firmware version of the device identified by
     * the specified ID. This can be useful for compatibility checks and troubleshooting.
     *
     * @param id   The unique identifier of the device whose firmware version will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the firmware version number on success, or a negative error code on failure.
     */
    uint8_t readRawI2CAddress(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the stall protection status of the specified device.
     *
     * This function retrieves the current status of the stall protection feature for
     * the device identified by the specified ID. Stall protection helps prevent damage
     * due to motor stalls.
     *
     * @param id   The unique identifier of the device whose stall protection status will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the stall protection status on success, or a negative error code on failure.
     */
    int8_t readStallProtect(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the button status of the specified device.
     *
     * This function retrieves the current state of a button on the device identified
     * by the specified ID. The button state may indicate whether it is pressed or not.
     *
     * @param id   The unique identifier of the device whose button status will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the button state (e.g., pressed or not pressed) on success,
     *         or a negative error code on failure.
     */
    int8_t readButton(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the error code from the specified device.
     *
     * This function retrieves the current error code reported by the device identified
     * by the specified ID. This can be useful for diagnosing issues with the device's
     * operation.
     *
     * @param id   The unique identifier of the device whose error code will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the current error code on success, or a negative error code on failure.
     */
    int8_t readErrorCode(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the system status of the specified device.
     *
     * This function retrieves the current system status of the device identified by
     * the specified ID. The system status may include various operational states.
     *
     * @param id   The unique identifier of the device whose system status will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the system status value on success, or a negative error code on failure.
     */
    int8_t readSysStatus(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the motor ID of the specified device.
     *
     * This function retrieves the unique identifier of the motor associated with the
     * device identified by the specified ID. The motor ID can be used for reference
     * in applications where multiple motors are used.
     *
     * @param id   The unique identifier of the device whose motor ID will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the motor ID on success.
     */
    uint8_t readMotorID(uint8_t id, uint8_t addr);

    /**
     * @brief Reads the BPS (Bits Per Second) setting of the specified device.
     *
     * This function retrieves the current BPS setting of the device identified by
     * the specified ID. This setting indicates the communication speed for data transmission.
     *
     * @param id   The unique identifier of the device whose BPS setting will be read.
     *             Valid range: 0 to 255.
     * @param addr The I2C address (0-127) of the device to communicate with.
     *
     * @return Returns the BPS value on success, or a negative error code on failure.
     */
    int8_t readBPS(uint8_t id, uint8_t addr);

private:
    int8_t txNum;
    int8_t rxNum;
    bool mutexLocked;
    uint8_t recv_motorId;
    uint8_t recv_faultInfo;
    uint8_t recv_mode;
    bool recv_status;
    uint8_t canData[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t readData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};
#endif