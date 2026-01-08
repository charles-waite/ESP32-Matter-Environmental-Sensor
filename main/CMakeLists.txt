idf_component_register(
  SRCS "MainSensor.cpp"
  INCLUDE_DIRS "."
  REQUIRES
    arduino-esp32
    bsec2
    BME68x_Sensor_library
    oled_ssd1306_tp
    qrcodeoled
    esp32_matter_extra_endpoints
)