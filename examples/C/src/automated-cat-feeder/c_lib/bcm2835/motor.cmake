target_sources(${LF_MAIN_TARGET} PRIVATE
    HR8825.c
    DEV_Config.c
)
target_link_libraries(${LF_MAIN_TARGET} PRIVATE bcm2835 m)

