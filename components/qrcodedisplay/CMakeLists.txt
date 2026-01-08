file(GLOB SRCS "src/*.c" "src/*.cpp")

idf_component_register(
  SRCS ${SRCS}
  INCLUDE_DIRS "src"
  REQUIRES arduino-esp32
)

# Some versions assume Arduino/ESP32 macros exist
target_compile_definitions(${COMPONENT_LIB} PUBLIC
  ARDUINO=1
  ESP32=1
  ARDUINO_ARCH_ESP32=1
)