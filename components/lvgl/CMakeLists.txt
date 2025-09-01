file(GLOB_RECURSE SOURCES
    "lvgl/src/core/*.c"
    "lvgl/src/draw/*.c"
    "lvgl/src/font/*.c"
    "lvgl/src/hal/*.c"
    "lvgl/src/misc/*.c"
    "lvgl/src/widgets/*.c"
    "lvgl/src/extra/*.c"
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "." "lvgl/src" "lvgl"
    REQUIRES esp_driver_spi
)