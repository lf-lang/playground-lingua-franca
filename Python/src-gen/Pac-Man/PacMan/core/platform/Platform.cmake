# Check which system we are running on to select the correct platform support
# file and assign the file's path to LF_PLATFORM_FILE
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(LF_PLATFORM_FILE lf_linux_support.c)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    set(LF_PLATFORM_FILE lf_macos_support.c)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(LF_PLATFORM_FILE lf_windows_support.c)
    set(CMAKE_SYSTEM_VERSION 10.0)
    message("Using Windows SDK version ${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}")
else()
    message(FATAL_ERROR "Your platform is not supported! The C target supports Linux, MacOS and Windows.")
endif()
