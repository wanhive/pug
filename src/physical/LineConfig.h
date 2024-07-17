/*
 * LineConfig.h
 *
 * Copyright (C) 2023 Amit Kumar (amitkriit@gmail.com)
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

#ifndef WH_PHYSICAL_LINECONFIG_H_
#define WH_PHYSICAL_LINECONFIG_H_

namespace wanhive {

/**
 * Possible direction settings
 */
enum class LineDirection {
	NONE, /**< IO disabled */
	INPUT, /**< Reading the GPIO line */
	OUTPUT,/**< Driving the GPIO line */
	BOTH /**< Bidirectional */
};

/**
 * Possible active state settings
 */
enum class LineState {
	ACTIVE_HIGH,/**< Physical high (default) */
	ACTIVE_LOW /**< Physical low */
};

/**
 * Possible driver settings
 */
enum class LineDrive {
	NONE, /**< Default (push-pull) */
	OPEN_DRAIN,/**< Open-drain output */
	OPEN_SOURCE/**< Open-source output */
};

/**
 * Possible line bias settings
 */
enum class LineBias {
	NONE, /**< Default (as-is) */
	PULL_UP, /**< Pull-up bias enabled */
	PULL_DOWN,/**< Pull-down bias enabled */
	DISABLED /**< Bias disabled */
};

/**
 * Possible line events for monitoring
 */
enum class LineEvent {
	NONE, /**< Not known/applicable */
	RISING, /**< Rising edge */
	FALLING,/**< Falling edge */
	BOTH /**< Both (rising and falling) edges */
};

/**
 * Possible clock types for line event time-stamp
 */
enum class LineClock {
	NONE,/**< Default (monotonic clock) or don't care */
	REALTIME /**< Use realtime clock */
};

/**
 * Debounce settings
 */
struct LineDebounce {
	bool on { false }; // enable or disable
	unsigned int period { 0 }; //debounce period in microseconds
};

/**
 * Line configuration
 */
struct LineConfig {
	LineDirection direction { LineDirection::INPUT };
	LineState state { LineState::ACTIVE_HIGH };
	LineDrive drive { LineDrive::NONE };
	LineBias bias { LineBias::NONE };
	LineEvent event { LineEvent::NONE };
	LineClock clock { LineClock::NONE };
	LineDebounce debounce;
	bool busy { false };
};

}  // namespace wanhive

#endif /* WH_PHYSICAL_LINECONFIG_H_ */
