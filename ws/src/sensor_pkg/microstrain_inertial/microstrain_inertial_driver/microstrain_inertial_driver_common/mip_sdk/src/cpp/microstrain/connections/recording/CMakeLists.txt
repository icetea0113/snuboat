set(MICROSTRAIN_RECORDING_CONNECTION_LIBRARY "microstrain_recording_connection" CACHE INTERNAL "Name of the MicroStrain recording connections library")
mark_as_advanced(MICROSTRAIN_RECORDING_CONNECTION_LIBRARY)

set(MICROSTRAIN_RECORDING_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/recording_connection.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/recording_connection.hpp"
    "${MICROSTRAIN_CONNECTION_SOURCES}"
)

add_library(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY}
    "${MICROSTRAIN_RECORDING_SOURCES}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_RECORDING_SOURCES})
endif()

target_compile_features(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY} PUBLIC cxx_std_11)

target_link_libraries(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY} PUBLIC ${MICROSTRAIN_LIBRARY})

target_compile_options(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

set(MICROSTRAIN_LIBRARIES_TMP ${MICROSTRAIN_LIBRARIES_TMP} ${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MICROSTRAIN_RECORDING_CONNECTION_LIBRARY} ${MICROSTRAIN_SRC_DIR})
