# --- SOURCE FILES DECLARATIONS START ---

## src/physical
WH_PHY_HDRS = physical/GPIO.h physical/I2C.h physical/LineConfig.h \
	physical/SMBus.h physical/SPI.h physical/Terminal.h \
	physical/TerminalConfig.h physical/UART.h
WH_PHY_SRCS = physical/GPIO.cpp physical/I2C.cpp physical/SMBus.cpp \
	physical/SPI.cpp physical/Terminal.cpp physical/TerminalConfig.cpp \
	physical/UART.cpp

## src/driver
WH_DRIV_HDRS = driver/ADS111x.h driver/PCA9685.h
WH_DRIV_SRCS = driver/ADS111x.cpp driver/PCA9685.cpp

## src/
WH_PUG_TOP_HDRS = wanhive-pug.h

# --- SOURCE FILES DECLARATIONS END ---

lib_LTLIBRARIES = libwanhivepug.la
libwanhivepug_la_CXXFLAGS = -Wall
# https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
libwanhivepug_la_LDFLAGS = -version-info @WH_LIB_VERSION@
libwanhivepug_la_SOURCES = $(WH_PUG_TOP_HDRS) $(WH_PHY_HDRS) $(WH_DRIV_HDRS) \
	$(WH_PHY_SRCS) $(WH_DRIV_SRCS)
libwanhivepug_la_LIBADD= -lwanhive -li2c

# --- Library headers for installation ---
wanhivepugtopdir=$(includedir)/wanhive
wanhivepugtop_HEADERS=$(WH_PUG_TOP_HDRS)

wanhivepugphydir=$(includedir)/wanhive/physical
wanhivepugphy_HEADERS=$(WH_PHY_HDRS)
wanhivepugdrivdir=$(includedir)/wanhive/driver
wanhivepugdriv_HEADERS=$(WH_DRIV_HDRS)

pkgconfig_DATA = wanhivepug.pc
