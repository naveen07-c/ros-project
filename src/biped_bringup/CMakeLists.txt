cmake_minimum_required(VERSION 3.8)
project(biped_bringup)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(biped_cad_playback SHARED src/cad_playback.cpp)
target_compile_features(biped_cad_playback PRIVATE cxx_std_17)
target_link_libraries(biped_cad_playback gz-sim8::gz-sim8)
install(TARGETS biped_cad_playback LIBRARY DESTINATION lib)

install(
  DIRECTORY worlds launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
