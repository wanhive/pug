/*
 * wanhive-pug.h
 *
 * Copyright (C) 2024 Wanhive Systems Private Limited (info@wanhive.com)
 *
 * SPDX License Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef WH_PUG_H_
#define WH_PUG_H_

/*
 * Physical computing interfaces
 */
#include "physical/GPIO.h"
#include "physical/SPI.h"
#include "physical/UART.h"

/*
 * Device drivers
 */
#include "driver/ADS111x.h"
#include "driver/BME280.h"
#include "driver/BME68x.h"
#include "driver/BMI270.h"
#include "driver/BMM350.h"
/*
 * Peripheral controllers
 */
#include "device/Gimbal.h"

#endif /* WH_PUG_H_ */
