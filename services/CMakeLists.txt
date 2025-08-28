
add_library(services STATIC
    at_engine/src/at_engine.c
)
target_include_directories(services PUBLIC at_engine/include)
target_link_libraries(services PUBLIC core)
