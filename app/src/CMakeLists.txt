cmake_minimum_required(VERSION 3.20)
project(utils_lib CXX)

set(SOURCE_FILES
    all.hpp
    bitmask.hpp
    bitset.hpp
    bimap.hpp
    template_string.hpp
    optional.hpp
)

add_library(utils_lib SHARED STATIC ${SOURCE_FILES})

set_target_properties(utils_lib PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS utils_lib DESTINATION ${UTILS_INSTALL_LIB_DIR})
install(FILES all.hpp bitmask.hpp bitset.hpp bimap.hpp template_string.hpp optional.hpp DESTINATION ${UTILS_INSTALL_INCLUDE_DIR})