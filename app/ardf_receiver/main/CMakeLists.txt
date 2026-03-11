idf_component_register(SRCS "main.cc"
                            "logger.cc"
                            "util.cc"
                            "receiver_system.cc"
                            "task.cc"
                            "i2c_util.cc"
                            "adc_util.cc"
                            "si5351a.cc"
                            "mcp4018.cc"
                            "ssd1306.cc"
                            "ui_controller_task.cc"
                            "receiver_settings.cc"
                            "battery_monitor_task.cc"
                            "signal_meter_task.cc"
                    INCLUDE_DIRS "")

component_compile_options(-Wno-error=format= -Wno-format)

set(COMPONENTS esp32 main)

