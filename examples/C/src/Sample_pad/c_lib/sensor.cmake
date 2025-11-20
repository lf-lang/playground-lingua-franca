file(GLOB_RECURSE BME280_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/BME280_SensorAPI/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/BME280_SensorAPI/*.h"
)

foreach(FILE_PATH ${BME280_SRC})
    if(FILE_PATH MATCHES ".*/BME280_SensorAPI/examples/.*")
        list(REMOVE_ITEM BME280_SRC ${FILE_PATH})
    endif()
endforeach()

target_sources(${LF_MAIN_TARGET} PRIVATE 
    bme_utils.h
    bme_utils.c
    ${BME280_SRC}
    bh_utils.h
    bh_utils.c
)

target_include_directories(${LF_MAIN_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BME280_SensorAPI
)