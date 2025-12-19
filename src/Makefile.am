# --- SOURCE FILES DECLARATIONS START ---

## src/physical
WH_PHY_HDRS = physical/GPIO.h physical/I2C.h physical/LineConfig.h \
	physical/SMBus.h physical/SPI.h physical/Terminal.h \
	physical/TerminalConfig.h physical/UART.h physical/UARTConfig.h
WH_PHY_SRCS = physical/GPIO.cpp physical/I2C.cpp physical/SMBus.cpp \
	physical/SPI.cpp physical/Terminal.cpp physical/TerminalConfig.cpp \
	physical/UART.cpp

## src/driver
WH_DRIV_HDRS = driver/ADS111x.h driver/BME280.h driver/BME68x.h \
	driver/BME69x.h driver/BMI270.h driver/BMM350.h driver/MLX9064x.h \
	driver/MLX90640.h driver/PCA9685.h
WH_DRIV_SRCS = driver/ADS111x.cpp driver/BME280.cpp driver/BME68x.cpp \
	driver/BME69x.cpp driver/BMI270.cpp driver/BMM350.cpp driver/MLX9064x.cpp \
	driver/MLX90640.cpp driver/PCA9685.cpp

## src/device
WH_DEVICE_HDRS = device/GeoLocation.h device/Gimbal.h device/GPS.h \
	device/Image.h device/Jpeg.h device/LogBook.h device/PWM.h \
	device/ServoControl.h device/UltraSonic.h device/ViewFinder.h
WH_DEVICE_SRCS = device/Gimbal.cpp device/GPS.cpp device/Jpeg.cpp \
	device/LogBook.cpp device/PWM.cpp device/ServoControl.cpp \
	device/UltraSonic.cpp device/ViewFinder.cpp

## src/
WH_PUG_HDRS = wanhive-pug.h

# --- SOURCE FILES DECLARATIONS END ---

AM_CXXFLAGS = -Wall -Werror

lib_LTLIBRARIES = libwanhive-pug.la
libwanhive_pug_la_CXXFLAGS = $(AM_CXXFLAGS) @WHUB_CFLAGS@ @LIBCAM_CFLAGS@
# https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
libwanhive_pug_la_LDFLAGS = -version-info @WH_LIB_VERSION@
libwanhive_pug_la_SOURCES = $(WH_PUG_HDRS) $(WH_PHY_HDRS) $(WH_DRIV_HDRS) \
	$(WH_DEVICE_HDRS) $(WH_PHY_SRCS) $(WH_DRIV_SRCS) $(WH_DEVICE_SRCS)
libwanhive_pug_la_LIBADD = @WHUB_LIBS@ @LIBCAM_LIBS@

# --- Library headers ---
# https://www.gnu.org/software/automake/manual/html_node/Uniform.html
# https://www.gnu.org/software/automake/manual/html_node/Headers.html
pugincludedir = $(includedir)/wanhive
nobase_puginclude_HEADERS = $(WH_PUG_HDRS) $(WH_PHY_HDRS) $(WH_DRIV_HDRS) \
	$(WH_DEVICE_HDRS)

# --- pkg-config metadata file ---
pkgconfig_DATA = wanhive-pug.pc
