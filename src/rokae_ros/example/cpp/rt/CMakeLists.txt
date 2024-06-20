#-------------------------------------------------------------------------------
# RT mode example executables setup

# Here puts required example source files
set(RT_EXAMPLES
    joint_position_control
    joint_impedance_control
    cartesian_impedance_control
    move_commands
    joint_s_line
    cartesian_s_line
    rt_industrial
    )

foreach(example ${RT_EXAMPLES})
    add_executable(${example} ${example}.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp)
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR})
    target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
    if(MSVC)
        add_compile_definitions(_USE_MATH_DEFINES)
    endif() # for M_PI in cmath
endforeach()

# check for xMateModel library option
if(XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        torque_control
        follow_joint_position
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} ${example}.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp)
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR})
        target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

#-------------------------------------------------------------------------------
# Installation
install(TARGETS ${RT_EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
