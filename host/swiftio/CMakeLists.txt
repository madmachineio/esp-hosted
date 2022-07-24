PROJECT(ESP32C3HOSTED)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../externlib/lib) 

include(./chal_export/flags.cmake)

SET(CMAKE_C_COMPILER arm-zephyr-eabi-gcc)
#SET(CMAKE_C_COMPILER gcc)

INCLUDE_DIRECTORIES(./common/)
INCLUDE_DIRECTORIES(./chal_export/include/)
INCLUDE_DIRECTORIES(./driver/netif/)
INCLUDE_DIRECTORIES(./driver/network/)
INCLUDE_DIRECTORIES(./driver/serial/)
INCLUDE_DIRECTORIES(./driver/spi/)
INCLUDE_DIRECTORIES(./driver/vhci/)
INCLUDE_DIRECTORIES(../host_common/include/)
INCLUDE_DIRECTORIES(../../common/include/)
INCLUDE_DIRECTORIES(../../common/protobuf-c/)

ADD_COMPILE_OPTIONS(${C_FLAGS})

file(GLOB_RECURSE LIBESP32C3HOSTED_SRC ./*.c)
	
ADD_LIBRARY(esp32c3hosted STATIC ${LIBESP32C3HOSTED_SRC}
									../host_common/transport_pserial.c
									../../common/protobuf-c/protobuf-c/protobuf-c.c
									../../common/esp_hosted_config.pb-c.c)

#add_custom_command(TARGET esp32c3hosted POST_BUILD
#	COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/../externlib/inc
#    COMMAND cp ${PROJECT_SOURCE_DIR}/mad_mp3dec.h ${PROJECT_SOURCE_DIR}/../externlib/inc)

#gcc ../libmad-0.15.1b/minimad.c -I../libmad-0.15.1b/ -L../../externlib/lib/ -lmadmp3decoder -o minimad
#./minimad < ~/test.mp3 1>test.pcm