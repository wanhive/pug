# Wanhive Pug

Wanhive Pug is a secure and high-performance physical computing library coded in C++.

This program is a part of the Wanhive IoT Platform.

## Features

Control GPIO pins and peripherals on single-board computers (SBCs) from user space.
 
- Digital I/O pins
- I2C
- SPI
- Terminal and UART

## Dependencies

- I2C userland development library (`libi2c`).
- Wanhive Hub [development library](https://github.com/wanhive/hub)

# Installation

Download the source archive.

```
tar -xvzf <archive-name>
./configure
make
make install
```

**NOTE:** Adjust the `PKG_CONFIG_PATH` environment variable if you installed the dependencies in a *non-standard* prefix.

# Resources

* [CHANGELOG](ChangeLog.md)
