set(GENERAL_SOURCES vector.c pqueue.c util.c)
set(MULTITHREADED_SOURCES semaphore.c)
add_sources_to_parent(GENERAL_SOURCES MULTITHREADED_SOURCES "")
