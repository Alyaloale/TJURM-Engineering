file(GLOB_RECURSE
    mining_tank_src
        ${CMAKE_SOURCE_DIR}/src/mining_tank/*.cpp
)


add_library(mining_tank
    STATIC
        ${mining_tank_src}
)

target_link_libraries(mining_tank
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
)
