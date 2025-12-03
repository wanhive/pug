# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## Changed

- **LogBook** interface improvements.
- Split up the header files for cleaner code.

## [0.12.0] - 2025-10-21

### Added

- Ultrasonic sensor driver.
- BME69x gas sensor driver.
- Sensor data logger.
- Finer break condition control of a terminal device.

### Changed

- BME68x gas sensor driver data structures and APIs.

## [0.11.0] - 2025-07-05

### Added

- Raw RGB data to JPEG conversion.
- Drive strength control function in the **BMI270** driver.
- PWM control function in the **PCA9685** driver.

### Changed

- **PWM** and **Gimbal** controllers re-implementation.
- Increase the pulse width parameter's precision from milliseconds to microseconds in **PWM::servo**.
- Update the **UART driver** to make it compatible with the I/O multiplexer.

### Fixed

- **BMM350** driver initialization and soft-reset errors.
- Allow full [0% - 100%] range for JPEG image quality.
- **I2C::resetAll** member function should have const modifier.

## [0.10.0] - 2025-05-15

### Added

- MLX90640 driver: differentiate between *no pixel defects* and *within threshold*.

## [0.9.0] - 2025-05-12

### Added

- Bare-bone USB/MIPI camera driver (based on `libcamera`).
- GPS tracker (relies on the GPS service daemon).

## [0.8.0] - 2025-04-29

### Changed

- Improve type and range safety in BME280 (environment sensor) driver.
- Improve type and range safety in BME68x (gas sensor) driver.

## [0.7.0] - 2025-04-26

### Added

- BMI270 inertial measurement unit driver.
- MLX90640 IR thermal camera driver.

### Changed

- BMM350 driver's enumerations and constants names.

## [0.6.0] - 2025-03-21

### Added

- BMM350 magnetometer driver.

## [0.5.0] - 2024-08-26

### Added

- **PCA9685::getFrequency** method to read the output modulation frequency value (Hz).
- **PWM** class for digital, pwm, and servo outputs.

### Changed

- 3-axis Gimbal controller re-implementation.

### Removed

- Redundant Servo class (replaced with the PWM class which offers better features).

## [0.4.0] - 2024-08-25

### Added

- **I2C::select** methods for device address selection.

## [0.3.0] - 2024-08-23

### Added

- 3-axis servo gimbal controller.

### Changed

- PCA9685 driver update.
- Replace the magic constants with symbolic names.

## [0.2.0] - 2024-08-11

### Added

- Analog to digital converter (ADS111x).
- Environment sensors (BME280, BME68x).

## [0.1.0] - 2024-07-18

### Added

- Initial implementation of the physical computing library in C++.
