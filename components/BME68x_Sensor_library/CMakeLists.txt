idf_component_register(
  SRCS
    "src/bme68x/bme68x.c"
    "src/bme68xLibrary.cpp"
  INCLUDE_DIRS
    "src"
    "src/bme68x"
  REQUIRES
    arduino-esp32
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
  ARDUINO=1
  ESP32=1
  ARDUINO_ARCH_ESP32=1
)