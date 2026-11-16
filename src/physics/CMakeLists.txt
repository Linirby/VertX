add_library(lili_physics OBJECT
    dynamic_body_system.cpp
    gravity_system.cpp
)
add_library(lili::physics ALIAS lili_physics)

target_include_directories(lili_physics
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

target_link_libraries(lili_physics PUBLIC lili_core lili_ecs)
