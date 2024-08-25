# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **PCA9685::getFrequency** method to read the output modulation frequency value (Hz).

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
