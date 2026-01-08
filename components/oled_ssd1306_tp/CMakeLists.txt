file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
  SRCS ${SRCS}
  INCLUDE_DIRS "src"
  REQUIRES arduino-esp32
)

# Force the OS/platform defines the library expects
target_compile_definitions(${COMPONENT_LIB} PUBLIC
  ARDUINO=1
  ESP32=1
)