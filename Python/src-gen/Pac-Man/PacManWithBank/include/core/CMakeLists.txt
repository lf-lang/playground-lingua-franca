set(CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Appends the provided sources to the SOURCES variable of the parent.
macro(add_sources_to_parent GENERAL_SOURCES MULTITHREADED_SOURCES SINGLE_THREADED_SOURCES)
    set(TEMP ${${GENERAL_SOURCES}})
    if(NUMBER_OF_WORKERS)
        list(APPEND TEMP ${${MULTITHREADED_SOURCES}})
    else()
        list(APPEND TEMP ${${SINGLE_THREADED_SOURCES}})
    endif(NUMBER_OF_WORKERS)
    foreach(FILE ${TEMP})
        file(RELATIVE_PATH RELATIVE_TO_ROOT ${CORE_ROOT} ${CMAKE_CURRENT_SOURCE_DIR}/${FILE})
        list(APPEND SOURCES ${RELATIVE_TO_ROOT})
    endforeach()
    set(SOURCES ${SOURCES} PARENT_SCOPE)
endmacro()

# Applies the variable whose name is X as a preprocessor definition,
#  if that variable is defined.
macro(define X)
    if(DEFINED ${X})
        message(STATUS ${X}=${${X}})
        add_compile_definitions(${X}=${${X}})
    endif(DEFINED ${X})
endmacro()

message(STATUS "Applying preprocessor definitions...")
define(_LF_CLOCK_SYNC_INITIAL)
define(_LF_CLOCK_SYNC_PERIOD_NS)
define(_LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL)
define(_LF_CLOCK_SYNC_ATTENUATION)
define(_LF_CLOCK_SYNC_COLLECT_STATS)
define(_LF_CLOCK_SYNC_ON)
define(LOG_LEVEL)
define(FEDERATED)
define(FEDERATED_CENTRALIZED)
define(FEDERATED_DECENTRALIZED)
define(NUMBER_OF_FEDERATES)
define(NUMBER_OF_WORKERS)
define(TARGET_FILES_DIRECTORY)
define(ADVANCE_MESSAGE_INTERVAL)
define(LINGUA_FRANCA_TRACE)
message(STATUS "")

# Add sources from subdirectories.
add_subdirectory(platform)
add_subdirectory(utils)

# List sources in this directory.
set(SINGLE_THREADED_SOURCES reactor.c)

# Determine which sources to include.
list(APPEND SOURCES ${GENERAL_SOURCES})
if(NUMBER_OF_WORKERS)
    message(STATUS "Including sources for threaded runtime with ${NUMBER_OF_WORKERS} worker(s).")
    add_subdirectory(threaded)
else()
    message(STATUS "Including sources for unthreaded runtime.")
    list(APPEND SOURCES ${SINGLE_THREADED_SOURCES})
endif(NUMBER_OF_WORKERS)

# Create the library "core".
list(JOIN SOURCES ", " PRINTABLE_SOURCE_LIST)
message(STATUS "Including the following sources: " ${PRINTABLE_SOURCE_LIST})
add_library(core ${SOURCES})

# Link threading library if necessary.
if(NUMBER_OF_WORKERS)
    find_package(Threads REQUIRED)
    target_link_libraries(${CoreLib} Threads::Threads)
endif(NUMBER_OF_WORKERS)
