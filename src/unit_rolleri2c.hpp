/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __UNIT_ROLLERI2C_H
#define __UNIT_ROLLERI2C_H

#include "Arduino.h"
#include "Wire.h"
#include "unit_roller_common.hpp"

class UnitRollerI2C {
public:
    /**
     * @brief Initializes the UnitRollerI2C object with I2C communication settings.
     *
     * This function configures the I2C bus with specified SDA and SCL pins, sets the clock speed,
     * and checks for communication with the device at the given address. It returns true if
     * the device responds to the initial transmission; otherwise, it returns false.
     *
     * @param wire Pointer to a TwoWire object used for I2C communication.
     * @param addr The I2C address of the device to communicate with.
     * @param sda The GPIO pin number used for the SDA line.
     * @param scl The GPIO pin number used for the SCL line.
     * @param speed The clock speed for the I2C bus in Hertz.
     *
     * @return True if the initialization was successful and the device is responsive,
     *         false otherwise.
     *
     * @note It is recommended to call this function before atdatating any other
     *       communication with the device. A delay is included after setting up
     *       the I2C configuration to allow the device time to stabilize.
     */
    bool begin(TwoWire *wire = &Wire, uint8_t addr = I2C_ADDR, uint8_t sda = 21, uint8_t scl = 22,
               uint32_t speed = 4000000L);

    /**
     * @brief Sets the operational mode of the UnitRollerI2C device.
     *
     * This function writes a byte to the mode register of the
     * UnitRollerI2C device to configure its operational mode.
     * The desired operational mode is specified by the input parameter.
     *
     * @param mode A byte value representing the desired operational mode:
     *             - 1: Speed Mode
     *             - 2: Position Mode
     *             - 3: Current Mode
     *             - 4: Encoder Mode
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_MODE_REG.
     */
    void setMode(roller_mode_t mode);

    /**
     * @brief Enables or disables the output of the UnitRollerI2C device.
     *
     * This function writes a single byte to the output register of the
     * UnitRollerI2C device to control whether the output is enabled
     * or disabled. The value of the input parameter determines the state
     * of the output.
     *
     * @param en A byte value representing the desired output state:
     *            - 0: Disable output
     *            - 1: Enable output
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_OUTPUT_REG.
     */
    void setOutput(uint8_t en);

    /**
     * @brief Sets the speed of the UnitRollerI2C device.
     *
     * This function writes a 32-bit integer value to the speed register of the
     * UnitRollerI2C device, allowing the user to configure the motor's speed.
     * The speed value can be positive or negative, where positive values typically
     * indicate forward motion and negative values indicate reverse motion.
     *
     * @param speed A 32-bit integer representing the desired speed of the motor.
     *              The valid range may depend on the specific implementation and
     *              capabilities of the UnitRollerI2C device.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_SPEED_REG.
     */
    void setSpeed(int32_t speed);

    /**
     * @brief Sets the maximum current limit for speed control in the UnitRollerI2C device.
     *
     * This function writes a 32-bit integer value to the maximum current register of the
     * UnitRollerI2C device. This allows the user to set a limit on the current that
     * can be drawn by the motor when operating at maximum speed, which helps prevent
     * overheating or damage to the device.
     *
     * @param current A 32-bit signed integer representing the desired current limit.
     *                The valid range is from -120,000 to 120,000, which corresponds
     *                to the allowable current levels in milliamps.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_SPEED_MAX_CURRENT_REG.
     */
    void setSpeedMaxCurrent(int32_t current);

    /**
     * @brief Sets the PID (Proportional, Integral, Derivative) coefficients for speed control
     *        in the UnitRollerI2C device.
     *
     * This function writes three 32-bit unsigned integer values to the PID coefficient register of
     * the UnitRollerI2C device. These coefficients are used in the PID control algorithm
     * to regulate the speed of the motor, allowing for precise control based on the
     * current error, accumulated past errors, and the rate of change of error.
     *
     * @param p A 32-bit unsigned integer representing the proportional coefficient of the PID controller.
     *          This coefficient determines how aggressively the controller responds to the current error.
     *          For example: mottor P=0.004, speedP setting  value=0.004*100000=400,
     *
     * @param i A 32-bit unsigned integer representing the integral coefficient of the PID controller.
     *          This coefficient helps eliminate steady-state error by adjusting the output based
     *          on the accumulated error over time.
     *          For example: mottor I=0.002, speedI settingvalue=0.002*10000000=20000,
     *
     * @param d A 32-bit unsigned integer representing the derivative coefficient of the PID controller.
     *          This coefficient predicts future error based on its rate of change, providing a
     *          damping effect to reduce overshoot.
     *          For example: mottor D=16.00, speedD setting  value=16.00*100000=1600000,
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_SPEED_PID_REG.
     *       Each coefficient is sent in the following order: proportional, integral, and then derivative.
     */
    void setSpeedPID(uint32_t p, uint32_t i, uint32_t d);

    /**
     * @brief Sets the position of the UnitRollerI2C device.
     *
     * This function writes a 32-bit integer value to the position register of the
     * UnitRollerI2C device, allowing the user to configure the desired position
     * for operation. The position value can be positive or negative, depending on
     * the specific implementation and the context in which the device operates.
     *
     * @param pos A 32-bit integer representing the desired position of the device.
     *            The valid range for the position may depend on the specific use case
     *            and capabilities of the UnitRollerI2C device.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_POS_REG.
     */
    void setPos(int32_t pos);

    /**
     * @brief Sets the maximum current limit for position control in the UnitRollerI2C device.
     *
     * This function writes a 32-bit integer value to the maximum current register of the
     * UnitRollerI2C device. This allows the user to set a limit on the current that
     * can be drawn by the motor when operating at maximum position, helping to prevent
     * overheating and ensuring safe operation of the device.
     *
     * @param current A 32-bit signed integer representing the desired current limit.
     *                The valid range is from -120,000 to 120,000, which corresponds
     *                to the allowable current levels in milliamps.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_POS_MAX_CURRENT_REG.
     */
    void setPosMaxCurrent(int32_t current);

    /**
     * @brief Sets the PID (Proportional, Integral, Derivative) coefficients for position control
     *        in the UnitRollerI2C device.
     *
     * This function writes three 32-bit unsigned integer values to the PID coefficient register of
     * the UnitRollerI2C device. These coefficients are used in the PID control algorithm
     * to accurately control the position of the motor, allowing for fine adjustments based on
     * the current positional error, accumulated past errors, and the rate of change of error.
     *
     * @param p A 32-bit unsigned integer representing the proportional coefficient of the PID controller.
     *          This coefficient determines how aggressively the controller responds to the current positional error.
     *          For example: mottor P=0.004, speedP setting  value=0.004*100000=400,
     *
     * @param i A 32-bit unsigned integer representing the integral coefficient of the PID controller.
     *          This coefficient helps eliminate steady-state error by adjusting the output based
     *          on the accumulated positional error over time.
     *          For example: mottor I=0.002, speedI settingvalue=0.002*10000000=20000,
     *
     * @param d A 32-bit unsigned integer representing the derivative coefficient of the PID controller.
     *          This coefficient predicts future positional error based on its rate of change, providing a
     *          damping effect to reduce overshoot when moving to the desired position.
     *          For example: mottor D=16.00, speedD setting  value=16.00*100000=1600000,
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_POS_PID_REG.
     *       Each coefficient is sent in the following order: proportional, integral, and then derivative.
     */
    void setPosPID(uint32_t p, uint32_t i, uint32_t d);

    /**
     * @brief Sets the current limit for the UnitRollerI2C device.
     *
     * This function writes a 32-bit integer value to the current register of the
     * UnitRollerI2C device. This allows the user to specify the desired current
     * level (in milliamps or appropriate units) that the device should maintain
     * during operation, helping to protect against overcurrent conditions.
     *
     * @param current A 32-bit signed integer representing the desired current limit.
     *                The valid range is from -120,000 to 120,000, which corresponds
     *                to the allowable current levels in milliamps.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_CURRENT_REG.
     */
    void setCurrent(int32_t current);

    /**
     * @brief Sets the dial counter value for the UnitRollerI2C encoder.
     *
     * This function writes a 32-bit integer value to the dial counter register of
     * the UnitRollerI2C device. The dial counter is used to track the position or
     * rotation count of the encoder, allowing for precise measurement and control of
     * angular displacement.
     *
     * @param counter A 32-bit signed integer representing the desired dial counter value.
     *                This value can be set to reflect the current position of the encoder
     *                and may range anywhere within the limits of a 32-bit signed integer.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_DIAL_COUNTER_REG.
     */
    void setDialCounter(int32_t counter);

    /**
     * @brief Sets the RGB mode for the UnitRollerI2C device.
     *
     * This function writes a value to the RGB mode register to configure the
     * device's operation mode for RGB lighting. The mode can be set to either
     * a system-default or user-defined configuration.
     *
     * @param mode A uint8_t value representing the desired RGB mode.
     *             Possible values are:
     *             - 0: System-default mode
     *             - 1: User-defined mode
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The RGB mode is set by writing to
     *       the register identified by UnitRollerI2C_RGB_REG + 3.
     */
    void setRGBMode(roller_rgb_t mode);

    /**
     * @brief Set the RGB color value.
     *
     * This function converts the specified color value to BGR format and writes it
     * to the appropriate register of the device.
     *
     * @param color The color value to set, typically an integer representing RGB color.
     *              If using a 16-bit RGB565 format, the color consists of 5 bits for red,
     *              6 bits for green, and 5 bits for blue. In this case, the color parameter
     *              should be provided in the correct format to ensure accurate extraction.
     *
     * - R: (color >> 11) & 0x1F (5 bits)
     * - G: (color >> 5) & 0x3F  (6 bits)
     * - B: color & 0x1F         (5 bits)
     *
     * The extracted R, G, and B components will be expanded to 8 bits for compatibility
     * with standard RGB values.
     *
     * @note Extraction and expansion algorithm:
     * - Red component is expanded to 8 bits by multiplying by 255 and dividing by 31.
     * - Green component is expanded to 8 bits by multiplying by 255 and dividing by 63.
     * - Blue component is expanded to 8 bits by multiplying by 255 and dividing by 31.
     *
     * Finally, the R, G, B values are stored in a byte array in BGR format and
     * written to the device via the writeBytes function.
     */
    void setRGB(int32_t color);

    /**
     * @brief Sets the brightness of the RGB LED on the UnitRollerI2C device.
     *
     * This function writes a byte to the RGB brightness register of the
     * UnitRollerI2C device, allowing the user to adjust the brightness
     * level of the RGB LED. The brightness level is specified by the
     * input parameter, which should be within the defined range.
     *
     * @param brightness A byte value representing the desired brightness level,
     *                   valid in the range of 0 to 100.
     *                   - 0: Off
     *                   - 100: Maximum brightness
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_RGB_BRIGHTNESS_REG.
     */
    void setRGBBrightness(uint8_t brightness);

    /**
     * @brief Sets the motor ID for the UnitRollerI2C device.
     *
     * This function writes a byte to the ID register of the
     * UnitRollerI2C device to configure its motor ID.
     * The motor ID is used to uniquely identify the motor within
     * a system that may have multiple motors connected.
     *
     * @param id A byte value representing the motor ID, valid in the range
     *           of 0 to 255. Each motor should have a unique ID within this range.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_ID_REG.
     */
    void setMotorID(uint8_t id);

    /**
     * @brief Sets the baud rate for the UnitRollerI2C device.
     *
     * This function writes a byte to the baud rate register of the
     * UnitRollerI2C device, allowing the user to configure the
     * communication speed. The specified baud rate is determined by
     * the input parameter.
     *
     * @param bps A byte value representing the desired baud rate:
     *            - 0: 115200 bps
     *            - 1: 19200 bps
     *            - 2: 9600 bps
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_BPS_REG.
     */
    void setBPS(roller_bps_t bps);

    /**
     * @brief Saves the most recent configuration changes of the UnitRollerI2C device to flash memory.
     *
     * This function writes a value to the designated flash save register to
     * trigger the storage of the latest configuration settings into non-volatile
     * flash memory. This ensures that all modifications made during the current
     * session are retained even when the device is powered off or reset.
     *
     * @note A single byte with the value of 1 is sent to the register
     *       (UnitRollerI2C_SAVE_FLASH_REG) to initiate the saving process.
     *       After the write operation, a delay of 100 milliseconds is included
     *       to allow sufficient time for the saving process to complete.
     *
     *       Ensure that I2C communication has been properly initialized
     *       before calling this function.
     */
    void saveConfigToFlash(void);

    /**
     * @brief Set the motor position range protection mechanism.
     *
     * This function enables or disables the position range protection mechanism
     * for the motor.
     *
     * @param en A uint8_t value to enable (1) or disable (0) the protection
     * mechanism.
     *           - 0: Disable
     *           - 1: Enable
     *
     * @details This function writes the specified value to a designated register
     * to control the position range protection of the motor during operation. The
     * protection mechanism helps prevent the motor from exceeding its safe
     * operating range, avoiding potential damage.
     *
     * @return void
     */
    void posRangeProtect(uint8_t en);
    /**
     * @brief Resets the stalled protection mechanism of the UnitRollerI2C device.
     *
     * This function writes a specific value to the stalled protection reset register
     * to clear any stall conditions that may have been detected by the device.
     * This is useful for re-enabling normal operation after a stall has occurred
     * and needs to be addressed.
     *
     * @note A single byte with the value of 1 is sent to the register
     *       (UnitRollerI2C_RESET_STALLED_PROTECT_REG) to initiate the reset
     *       of the stalled protection feature.
     *
     *       Ensure that I2C communication has been properly initialized
     *       before calling this function.
     */
    void resetStalledProtect(void);

    /**
     * @brief Configures the key switch mode on the UnitRollerI2C device.
     *
     * This function sets the key switch mode by writing a byte to the
     * corresponding register in the UnitRollerI2C device. It takes an
     * enable/disable value as input, which determines the mode of operation.
     *
     * @param en A byte value (0 or 1) indicating whether to enable (1)
     *           or disable (0) the key switch mode.
     *
     * @note The function assumes that the I2C communication has been properly
     *       initialized before calling this function. The specific register
     *       being written to is defined by the constant
     *       UnitRollerI2C_KEY_SWTICH_MODE_REG.
     */
    void setKeySwitchMode(uint8_t en);

    /**
     * @brief Configures the stall protection feature of the UnitRollerI2C device.
     *
     * This function writes a byte to the stall protection register of the
     * UnitRollerI2C device to enable or disable the stall protection feature.
     * This feature helps prevent motor stalls by monitoring the motor's operation
     * and taking corrective action when necessary.
     *
     * @param en A byte value indicating whether to enable (1) or disable (0)
     *           the stall protection feature.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being written
     *       to is identified by the constant UnitRollerI2C_STALL_PROTECTION_REG.
     */
    void setStallProtection(uint8_t en);

    /**
     * @brief Sets the I2C address for the UnitRollerI2C device.
     *
     * This function updates the I2C address used to communicate with the
     * UnitRollerI2C device. It sends the new address to the device over
     * the I2C bus, allowing for dynamic reconfiguration of communication
     * settings.
     *
     * @param addr The new I2C address to be set for the device. This should
     *             be a valid I2C address between 1 and 127.
     *
     * @return uint8_t The updated I2C address that was set for the device.
     *
     * @note Ensure that the specified address does not conflict with other
     *       devices on the I2C bus. It is also important to verify that
     *       the device is powered on and properly connected before calling
     *       this function.
     */
    uint8_t setI2CAddress(uint8_t addr);

    /**
     * @brief Retrieves the Speed PID parameters from the UnitRollerI2C device.
     *
     * This function reads the proportional (P), integral (I), and derivative (D)
     * values used in the speed PID control loop from the device's registers.
     * The retrieved values are stored in the provided pointers.
     *
     * @param p A pointer to a uint32_t variable where the proportional value will be stored.
     * @param i A pointer to a uint32_t variable where the integral value will be stored.
     * @param d A pointer to a uint32_t variable where the derivative value will be stored.
     *
     * @note The function reads 12 bytes of data starting from the register
     *       (UnitRollerI2C_SPEED_PID_REG). The first 4 bytes correspond to the P value,
     *       the next 4 bytes correspond to the I value, and the last 4 bytes correspond
     *       to the D value. Ensure that I2C communication has been properly initialized
     *       before calling this function.
     */
    void getSpeedPID(uint32_t *p, uint32_t *i, uint32_t *d);

    /**
     * @brief Retrieves the Position PID parameters from the UnitRollerI2C device.
     *
     * This function reads the proportional (P), integral (I), and derivative (D)
     * values used in the position PID control loop from the device's registers.
     * The retrieved values are then stored in the provided pointers.
     *
     * @param p A pointer to a uint32_t variable where the proportional value will be stored.
     * @param i A pointer to a uint32_t variable where the integral value will be stored.
     * @param d A pointer to a uint32_t variable where the derivative value will be stored.
     *
     * @note The function reads 12 bytes of data starting from the register
     *       (UnitRollerI2C_POS_PID_REG). The first 4 bytes correspond to the P value,
     *       the next 4 bytes correspond to the I value, and the last 4 bytes correspond
     *       to the D value. Ensure that I2C communication has been properly initialized
     *       before calling this function.
     */
    void getPosPID(uint32_t *p, uint32_t *i, uint32_t *d);

    /**
     * @brief Retrieves the RGB values from the Unit Roller I2C device.
     *
     * This function reads three bytes of RGB data from the specified
     * I2C address and stores the values in the provided pointers.
     * The RGB values are expected to be ordered as follows:
     * - R: data[2]
     * - G: data[1]
     * - B: data[0]
     *
     * @param r Pointer to a uint8_t variable where the red value will be stored.
     * @param g Pointer to a uint8_t variable where the green value will be stored.
     * @param b Pointer to a uint8_t variable where the blue value will be stored.
     *
     * @note Ensure that the pointers passed to this function are valid
     *       and point to allocated memory large enough to hold the values.
     */
    void getRGB(uint8_t *r, uint8_t *g, uint8_t *b);

    /**
     * @brief Retrieves the speed value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the desired or
     * commanded speed of the device. The retrieved value indicates the speed
     * setting that the device is configured to achieve.
     *
     * @return int32_t The speed value, as a signed 32-bit integer.
     *                 The value may represent speed in units such as RPM
     *                 (revolutions per minute) or another relevant measure,
     *                 depending on the device's specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_SPEED_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getSpeed(void);

    /**
     * @brief Retrieves the maximum current value permitted at the specified speed
     * from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the maximum
     * current that the device is allowed to draw when operating at its set speed.
     * This value can be useful for ensuring safe operation and preventing
     * overcurrent conditions.
     *
     * @return int32_t The maximum current value, as a signed 32-bit integer.
     *                 The value may represent current in units such as milliamps
     *                 (mA) or another relevant measure, depending on the device's
     *                 specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_SPEED_MAX_CURRENT_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    int32_t getSpeedMaxCurrent(void);

    /**
     * @brief Retrieves the actual speed readback value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current speed
     * of the device. The retrieved value indicates the actual speed being measured
     * at the moment and is returned to the caller.
     *
     * @return int32_t The actual speed value, as a signed 32-bit integer.
     *                 The value may represent speed in units such as RPM
     *                 (revolutions per minute) or another relevant measure,
     *                 depending on the device's specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_SPEED_READBACK_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getSpeedReadback(void);

    /**
     * @brief Retrieves the current position value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current position
     * of the device. The retrieved value indicates the position in units that are
     * defined by the device's specifications, which could be in counts, degrees,
     * or another relevant measure.
     *
     * @return int32_t The current position value, as a signed 32-bit integer.
     *                 The specific unit of measurement depends on the device
     *                 implementation and configuration.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_POS_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getPos(void);

    /**
     * @brief Retrieves the maximum current value permitted at the specified position
     * from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the maximum
     * current that the device is allowed to draw when operating at its set position.
     * This value is important for preventing overcurrent situations and ensuring safe
     * operation of the device.
     *
     * @return int32_t The maximum current value, as a signed 32-bit integer.
     *                 The value may represent current in units such as milliamps
     *                 (mA) or another relevant measure, depending on the device's
     *                 specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_POS_MAX_CURRENT_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    int32_t getPosMaxCurrent(void);

    /**
     * @brief Retrieves the real-time position value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current
     * position of the device in real time. The retrieved value provides
     * insight into the actual position where the device is currently located,
     * which may be useful for monitoring and control purposes.
     *
     * @return int32_t The real-time position value, as a signed 32-bit integer.
     *                 The specific unit of measurement depends on the device's
     *                 specifications, such as counts, degrees, or another relevant
     *                 measure.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_POS_READBACK_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getPosReadback(void);

    /**
     * @brief Retrieves the current value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current
     * flowing through the device. The retrieved value is stored in a dataorary
     * variable and returned to the caller.
     *
     * @return int32_t The current value, as a signed 32-bit integer.
     *                 The value may represent the current in milliamps (mA) or
     *                 another unit, depending on the device's specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_CURRENT_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getCurrent(void);

    /**
     * @brief Retrieves the real-time current value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current
     * flowing through the device in real time. The retrieved value provides
     * insight into the actual current consumption of the device, which is useful
     * for monitoring performance and ensuring safe operation.
     *
     * @return int32_t The real-time current value, as a signed 32-bit integer.
     *                 The specific unit of measurement may vary, commonly represented
     *                 in milliamps (mA) or another relevant measure, depending on
     *                 the device's specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_CURRENT_READBACK_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    int32_t getCurrentReadback(void);

    /**
     * @brief Retrieves the encoder value from the motor using the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current
     * position of the motor's encoder. The retrieved value is crucial for
     * tracking the motor's position or rotation count, which can be used
     * for precise control and feedback in various applications.
     *
     * @return int32_t The encoder value, as a signed 32-bit integer.
     *                 This value may represent counts, ticks, or another
     *                 relevant unit depending on the specific implementation
     *                 and configuration of the encoder.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_DIAL_COUNTER_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    int32_t getDialCounter(void);

    /**
     * @brief Retrieves the input voltage value from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the input
     * voltage (Vin) supplied to the device. The retrieved value is useful
     * for monitoring the power supply conditions and ensuring that the
     * device operates within acceptable voltage levels.
     *
     * @return int32_t The input voltage value, as a signed 32-bit integer.
     *                 The specific unit of measurement may vary, commonly
     *                 represented in volts (V) or millivolts (mV), depending
     *                 on the device's specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_VIN_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getVin(void);

    /**
     * @brief Retrieves the Temperature of the motor from the UnitRollerI2C device.
     *
     * This function reads a 32-bit integer value representing the current
     * Temperature of the motor. The retrieved value is essential for monitoring
     * the thermal conditions of the motor to prevent overheating and ensure safe
     * operation.
     *
     * @return int32_t The motor Temperature value, as a signed 32-bit integer.
     *                 The specific unit of measurement may vary, commonly
     *                 represented in degrees Celsius (°C) or another relevant
     *                 unit, depending on the device's specifications.
     *
     * @note The function reads 4 bytes of data starting from the register
     *       (UnitRollerI2C_TEMP_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    int32_t getTemp(void);

    /**
     * @brief Retrieves the system status of the motor from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current operating
     * status of the motor. The retrieved status can be used to determine
     * whether the motor is in standby, actively running, or experiencing an error.
     *
     * @return uint8_t The motor status:
     *         - 0: Standby
     *         - 1: Running
     *         - 2: Error
     *         Other values may represent different statuses depending on the
     *         specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_SYS_STATUS_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getSysStatus(void);

    /**
     * @brief Retrieves the error code from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current error state
     * of the motor. The retrieved error code can be used to diagnose issues
     * such as overvoltage or mechanical jams.
     *
     * @return uint8_t The motor error code:
     *         - 0: No error
     *         - 1: Overvoltage
     *         - 2: Jam
     *         Other values may represent different error states depending on
     *         the specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_ERROR_CODE_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getErrorCode(void);

    /**
     * @brief Get the current state of the motor position range protection
     * mechanism.
     *
     * This function reads the status of the position range protection mechanism
     * for the motor.
     *
     * @details The function retrieves the value from a designated register,
     * indicating whether the position range protection is enabled or disabled.
     * The returned value can be used to determine the current state of the
     * protection mechanism:
     *          - 0: Disabled
     *          - 1: Enabled
     *
     * @return uint8_t The current state of the position range protection (0 or
     * 1).
     */
    uint8_t getPosRangeProtect(void);

    /**
     * @brief Retrieves the current status of the stall protection feature
     *        from the UnitRollerI2C device.
     *
     * This function reads a byte from the stall protection register of the
     * UnitRollerI2C device to determine whether the stall protection feature
     * is enabled or disabled. The returned value indicates the current state.
     *
     * @return A byte value representing the stall protection status:
     *         0 if disabled, 1 if enabled.
     *
     * @note Ensure that I2C communication has been properly initialized
     *       before calling this function. The specific register being read
     *       from is defined by the constant UnitRollerI2C_STALL_PROTECTION_REG.
     */
    uint8_t getStallProtection(void);

    /**
     * @brief Retrieves the current key switch mode from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current key switch mode
     * setting. The retrieved value indicates whether the button switching mode is
     * enabled and how it operates during running mode.
     *
     * @return uint8_t The key switch mode:
     *         - 0: Off (button switching mode disabled)
     *         - 1: Press and hold for 5 seconds to switch modes while in running mode
     *         Other values may indicate different configurations depending on the
     *         specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_KEY_SWTICH_MODE_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getKeySwitchMode(void);

    /**
     * @brief Retrieves the output status of the motor from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current output
     * status of the motor. The retrieved status indicates whether the motor
     * is powered on or off.
     *
     * @return uint8_t The output status:
     *         - 0: Motor off
     *         - 1: Motor on
     *         Other values may represent different statuses depending on the
     *         specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_OUTPUT_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getOutputStatus(void);

    /**
     * @brief Retrieves the current operating mode of the motor from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current mode in which
     * the motor is operating. The retrieved mode can be used to understand
     * how the motor is configured for operation.
     *
     * @return uint8_t The motor mode:
     *         - 1: Speed Mode
     *         - 2: Position Mode
     *         - 3: Current Mode
     *         - 4: Encoder Mode
     *         Other values may represent different modes depending on the
     *         specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_MODE_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getMotorMode(void);

    /**
     * @brief Retrieves the ID of the motor from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the unique identifier
     * of the motor. The retrieved ID can be used for identification purposes,
     * especially in systems with multiple motors.
     *
     * @return uint8_t The motor ID, which is a unique identifier for the motor.
     *         The specific value will depend on the motor configuration and
     *         assignment.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_ID_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getMotorID(void);

    /**
     * @brief Retrieves the baud rate setting (BPS) from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current baud rate
     * configuration of the communication interface. The retrieved value indicates
     * the supported baud rates for communication.
     *
     * @return uint8_t The baud rate setting:
     *         - 0: 115200 bps
     *         - 1: 19200 bps
     *         - 2: 9600 bps
     *         Other values may represent different configurations based on the
     *         specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_BPS_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getBPS(void);

    /**
     * @brief Retrieves the brightness level of the RGB output from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current brightness setting
     * of the RGB LED. The retrieved value can be used to determine the intensity
     * of the RGB output.
     *
     * @return uint8_t The RGB brightness level, typically represented on a scale
     *         (e.g., 0-100) where:
     *         - 0: Minimum brightness (off)
     *         - 100: Maximum brightness (fully on)
     *         Other values may represent varying levels of brightness, depending
     *         on the device configuration.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_RGB_BRIGHTNESS_REG). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getRGBBrightness(void);

    /**
     * @brief Retrieves the current RGB mode from the UnitRollerI2C device.
     *
     * This function reads a single byte representing the current mode of
     * the RGB output. The retrieved value indicates whether the RGB settings
     * are using the system default or user-defined configurations.
     *
     * @return uint8_t The RGB mode:
     *         - 0: System-default mode
     *         - 1: User-defined mode
     *         Other values may represent different modes depending on the
     *         specific implementation of the device.
     *
     * @note The function reads 1 byte of data starting from the register
     *       (UnitRollerI2C_RGB_REG + 3). Ensure that I2C communication
     *       has been properly initialized before calling this function.
     */
    uint8_t getRGBMode(void);

    /**
     * @brief Retrieves the firmware version from the UnitRollerI2C device.
     *
     * This function sends a request to the device to read the current firmware
     * version, which can be useful for debugging and ensuring compatibility
     * with software features.
     *
     * @return uint8_t The firmware version as a single byte. The interpretation
     *         of this value will depend on the specific firmware versioning
     *         scheme used by the device manufacturer.
     *
     * @note The function initiates an I2C transmission to the specified address
     *       (_addr) and reads the firmware version from the register
     *       (FIRMWARE_VERSION_REG). Ensure that I2C communication has been
     *       properly initialized before calling this function.
     */
    uint8_t getFirmwareVersion(void);

    /**
     * @brief Retrieves the I2C address from the UnitRollerI2C device.
     *
     * This function sends a request to read the current I2C address
     * configuration of the device. The retrieved value can be useful for
     * understanding the communication setup, especially in multi-device
     * environments.
     *
     * @return uint8_t The I2C address of the device as configured. This value
     *         indicates the address used for I2C communication.
     *
     * @note The function initiates an I2C transmission to the specified address
     *       (_addr) and reads the I2C address from the register
     *       (I2C_ADDRESS_REG). Ensure that I2C communication has been properly
     *       initialized before calling this function.
     */
    uint8_t getI2CAddress(void);

private:
    uint8_t _addr;
    TwoWire *_wire;
    uint8_t _scl;
    uint8_t _sda;
    uint8_t _speed;

    /**
     * @brief Writes a sequence of bytes to a specific register on a device.
     *
     * This function begins a transmission to the specified I2C address, writes the register
     * address followed by a series of bytes from the provided buffer, and then ends the transmission.
     *
     * @param addr The I2C address of the device to communicate with.
     * @param reg The register address within the device where data will be written.
     * @param buffer A pointer to the array of bytes to be written to the device.
     * @param length The number of bytes to write from the buffer.
     *
     * @note If the transmission fails, you may not receive a proper acknowledgment
     *       from the device. Check the return status of endTransmission() if needed.
     */
    void writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);

    /**
     * @brief Reads a sequence of bytes from a specific register on a device.
     *
     * This function begins a transmission to the specified I2C address, writes the register
     * address from which data will be read, and then requests the specified number of bytes
     * from that device into the provided buffer.
     *
     * @param addr The I2C address of the device to communicate with.
     * @param reg The register address within the device from which data will be read.
     * @param buffer A pointer to the array where the read bytes will be stored.
     * @param length The number of bytes to read from the device.
     *
     * @note The function uses repeated start condition during transmission by passing
     *       'false' to endTransmission() to allow for reading without releasing the bus.
     *       Ensure that the device at the given address is ready to provide the requested data.
     */
    void readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);

    /**
     * @brief Converts a float value to an array of bytes.
     *
     * This function takes a floating-point number and converts it into a byte array representation.
     * It uses a union to reinterpret the float as an array of 4 bytes (assuming a standard 32-bit float).
     *
     * @param s The floating-point value to be converted.
     * @param d A pointer to the destination array where the byte representation will be stored.
     *
     * @note The caller is responsible for ensuring that the destination array has enough
     *       space (at least 4 bytes) to store the byte representation of the float.
     */
    float bytesToFloat(uint8_t *s);

    /**
     * @brief Converts an array of bytes to a floating-point value.
     *
     * This function takes an array of 4 bytes (assuming a standard 32-bit float)
     * and converts it into a floating-point number. It uses a union to reinterpret
     * the byte array as a float.
     *
     * @param s A pointer to the array of bytes representing the float value.
     *
     * @return The floating-point value represented by the input byte array.
     *
     * @note The caller must ensure that the input array contains at least 4 bytes.
     */
    void floatToBytes(float s, uint8_t *d);
};

#endif
