# Unit Roller Series

## Overview

### SKU: U182  U182-Lite

**Roller485 Unit** is an integrated brushless DC motor motion control kit designed for efficient motion control. It supports 6-16V DC power input (via PWR485 interface) or 5V input (via Grove interface) and can automatically adjust power coefficients to ensure optimal performance. It features a built-in **FOC** closed-loop drive system, utilizing a 3504 200KV brushless motor. Without forced cooling, it can operate with a maximum continuous phase current of 0.5A and a short-term current of 1A. The driver uses a magnetic encoder for feedback and supports current, speed, and position control, ensuring precise control.
Additionally, the device is equipped with a 0.66-inch **OLED** display on the back to display the device's status in real-time. It also includes an RGB indicator light and function button for easy human-computer interaction. The product's base is designed with LEGO-compatible mounting holes and M3 screw holes, making it easy to assemble and integrate. Roller485 Unit’s hardware and software are fully open-source, supporting motion control and parameter adjustment through **RS485** or **I2C** buses. It also offers SWD and SWO debugging interfaces, providing developers with greater flexibility. This product is widely used in robotic joints, motion control, industrial automation, and visual demonstration projects.

### SKU: U188  U188-Lite

**RollerCAN Unit** is a brushless DC motor motion control kit with multiple integrated control functions, designed for efficient motion control. It supports 6-16V DC power input (via the CAN (XT30) interface) or 5V input (via the Grove interface) and can automatically adjust the power factor to ensure optimal performance. It features a built-in **FOC** closed-loop drive system, using a 3504 200KV brushless motor. The maximum continuous operating phase current is 0.5A without forced cooling, and 1A for short-term. The driver uses a magnetic encoder for feedback and supports current, speed, and position triple-loop control, ensuring precise control. The unit’s shaft can optionally include a **slip ring**, allowing the Grove interface on the top to remain connected with the bottom, supporting 360° rotation while expanding additional modules at the top. This ensures power and data transmission through the rotating part.

Additionally, the unit is equipped with a 0.66-inch OLED display on the back, which can display the device status in real-time. It also includes an RGB indicator and function button for easy human-machine interaction. The top and base are designed with LEGO-compatible mounting holes and M3 screw holes, making it easy to build and integrate quickly. RollerCAN Unit's hardware and software are fully open-source, supporting motion control and parameter adjustment via  **CAN** or **I2C** buses, and it also provides SWD and SWO debugging interfaces, offering developers greater flexibility. This product is widely used in machine joints, motion control, industrial automation, and visual demonstration projects.

## Related Link

- [U182  Document & Datasheet](https://docs.m5stack.com/en/unit/Unit-Roller485)
- [U188  Document & Datasheet](https://docs.m5stack.com/en/unit/Unit-RollerCAN)

## Firmware Update over I2C

The library includes standalone serial-guided firmware update examples for ESP32 and ESP32-S3 boards. They use only
the Arduino `Wire` and `Serial` APIs, so an M5Stack host with a display is not required.

- `examples/i2c/firmware_update_485`: updates Roller485 to V2
- `examples/i2c/firmware_update_can`: updates RollerCAN to V3

Before uploading an example, set `kSdaPin` and `kSclPin` for the board's GROVE connector. Connect exactly one Unit
Roller, open the Serial Monitor at 115200 baud, and follow the confirmation prompt. The examples support both a normal
application update and recovery when the motor starts directly in its Bootloader. Do not remove power or disconnect
I2C while the update is running.

Both published legacy Bootloaders and current Bootloaders are supported through their common fixed `0x54` address
and ACK-based page-completion handshake. The updater deliberately avoids Bootloader reads because released legacy
Bootloaders cannot complete slave-read transactions.

Model mismatches, same-version reflashes, downgrades, unknown identities, and Bootloader recovery may be explicitly
authorized after showing a warning. This permits recovery when the wrong Roller application was previously installed.

Current firmware reports a product ID over I2C. Legacy firmware does not, and its communication ID may have been
customized, so the examples require explicit model confirmation whenever identity cannot be verified. A successful
update is followed by product and firmware-version verification.

When `UnitRollerI2C::begin()` detects legacy V1/V2 firmware that still uses the Roller485 (`0x00`) or RollerCAN
(`0xA8`) factory communication ID, it prints a yellow serial warning recommending the matching update example.
Customized IDs remain silent.

## License

- [MIT](LICENSE)
