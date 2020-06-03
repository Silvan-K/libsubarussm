# Add the shared library
add_library(subarussm ECUPort.cc)

# Add the example executable
add_executable(Example Example.cc)

# Link example executable to library
target_link_libraries(Example PUBLIC subarussm)
