find_package(Curses REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

target_link_libraries(${LF_MAIN_TARGET} ${CURSES_LIBRARY})
