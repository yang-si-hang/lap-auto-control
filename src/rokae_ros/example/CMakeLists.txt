#-------------------------------------------------------------------------------
# example executables setup

project(xCoreSDK-examples CXX)

# Here puts required example source files
set(EXAMPLES
    sdk_example
    move_example
    path_record
    rl_project
    read_robot_state
    )

# threading
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads REQUIRED)

foreach(example ${EXAMPLES})
    add_executable(${example} cpp/${example}.cpp cpp/print_helper.hpp)
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR})
    target_link_libraries(${example} Rokae::Rokae Threads::Threads eigen)
    if(MSVC)
        add_compile_definitions(_USE_MATH_DEFINES)
    endif() # for M_PI in cmath
endforeach()

# check for xMateModel library option
if(XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        xmatemodel_er3_er7p
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} cpp/${example}.cpp cpp/print_helper.hpp)
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR})
        target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

# if linking the shared library, add a macro for MSVC dllimport attribute
if(XCORE_LINK_SHARED_LIBS)
    add_compile_definitions(XCORESDK_DLL)
endif()

# ------------------------------------------------------------------------------
add_subdirectory(cpp/rt)

#-------------------------------------------------------------------------------
# Installation
install(TARGETS ${EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
