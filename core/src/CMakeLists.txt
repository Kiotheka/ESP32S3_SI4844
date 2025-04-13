idf_component_register(SRCS   
                              "main.cpp" 
                       INCLUDE_DIRS 
                              "."
                              "../inc"
                              "../../components/lvgl"
                              "../../components/eeprom"                              
                       REQUIRES
                              "freertos"                              
                              "driver"
                              "nvs_flash"
                              "esp_timer"                              
                              ) 
                       