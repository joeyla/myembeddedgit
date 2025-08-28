
add_library(core STATIC
    src/cbuf.c
    src/timebase.c
    src/log.c
)
target_include_directories(core PUBLIC include)
