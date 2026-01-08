idf_component_register(
  SRCS "src/qrcodeoled.cpp"
  INCLUDE_DIRS "src"
  REQUIRES oled_ssd1306_tp qrcodedisplay arduino-esp32
)