find_package(libwebsockets REQUIRED) # Finds the libwebsockets library

# Note that the PUBLIC keyword is needed otherwise CMake will issue an error
# saying that you must use either all-keywords or all-plain, and the keywords are previously used. 
target_link_libraries( ${LF_MAIN_TARGET} PUBLIC websockets ) # Links the websockets library
