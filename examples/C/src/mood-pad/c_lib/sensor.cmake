target_sources(${LF_MAIN_TARGET} PRIVATE 
    temp_utils.c
    light_utils.c
    capacitive_utils.c
    bme280.c
)

target_link_libraries(${LF_MAIN_TARGET} PRIVATE gpiod fluidsynth)

