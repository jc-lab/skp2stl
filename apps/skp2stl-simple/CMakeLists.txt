add_executable(skp2stl-simple main.cc)
target_link_libraries(skp2stl-simple PUBLIC SketchUpAPI skp2stl)
set_target_properties(
        skp2stl-simple
        PROPERTIES
        SUFFIX ".exe"
)

install(TARGETS skp2stl-simple)

