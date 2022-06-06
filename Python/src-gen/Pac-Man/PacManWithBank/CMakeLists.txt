set(SOURCES ctarget/schedule.c)

add_library(lib ctarget/schedule.c)
target_include_directories(lib PRIVATE ${PROJECT_SOURCE_DIR}/include/ctarget)
