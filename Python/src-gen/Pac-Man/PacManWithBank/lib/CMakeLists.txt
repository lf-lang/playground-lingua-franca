set(SOURCES schedule.c tag.c time.c util.c)

add_library(lib schedule.c tag.c time.c util.c)
target_include_directories(lib PRIVATE ${PROJECT_SOURCE_DIR}/include)
