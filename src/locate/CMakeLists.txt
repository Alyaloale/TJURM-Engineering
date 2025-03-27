file(GLOB_RECURSE
    locate_src
        ${CMAKE_SOURCE_DIR}/src/locate/*.cpp
        ${CMAKE_SOURCE_DIR}/src/locate/*.cu
)


add_library(locate
    STATIC
        ${locate_src}
)

target_link_libraries(locate
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
)
