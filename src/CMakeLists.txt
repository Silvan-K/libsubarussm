# Add the shared library
add_library(subarussm ECUPort.cc)

# Add the example executable
add_executable(SSMExample Example.cc)

# Link example executable to library
target_link_libraries(SSMExample PUBLIC subarussm)

# Install rules
install(TARGETS  subarussm DESTINATION lib/subarussm)
install(FILES ECUPort.hh Observables.hh Bytes.hh Exceptions.hh DESTINATION include/subarussm/)
install(TARGETS SSMExample DESTINATION bin/subarussm)
