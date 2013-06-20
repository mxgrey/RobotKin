cmake_minimum_required(VERSION 2.6)
project(RobotKin)

set( CMAKE_VERBOSE_MAKEFILE true )

include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB lib_source "src/*.cpp" "include/*.h" "parsing/*.cpp")
LIST(SORT lib_source)


file(GLOB unit_tests_source "test/*.cpp")
LIST(SORT unit_tests_source)

add_library(${PROJECT_NAME} SHARED ${lib_source})


# Check if the user is set up to parse URDFs
find_package( urdfdom )
if( urdfdom_FOUND )
   MESSAGE(STATUS "Found urdfdom -- We will compile a URDF parser")
   find_package( urdfdom_headers REQUIRED )
   include_directories( ${urdfdom_headers_INCLUDE_DIRS} )
   include_directories( ${urdfdom_INCLUDE_DIRS} )

   target_link_libraries( ${PROJECT_NAME}  ${urdfdom_LIBRARIES} )
   link_libraries( ${urdfdom_LIBRARIES} )

   MESSAGE(STATUS "${urdfdom_LIBRARIES}")

#   set(${urdfparser} "RobotKinUrdf")

#   file(GLOB parse_source "parsing/*.cpp")
#   LIST(SORT parse_source)
#   add_library(${urdfparser} SHARED ${parse_source})

#   target_link_libraries(${PROJECT_NAME} ${urdfparser})
#   link_libraries(${urdfparser})

   set_property( TARGET ${PROJECT_NAME}
                 PROPERTY COMPILE_DEFINITIONS "HAVE_URDF_PARSE" )

else( urdfdom_FOUND )
   MESSAGE(STATUS "Could NOT find urdfdom -- We will NOT compile a URDF parser")
endif( urdfdom_FOUND )
#############################################

enable_testing()

message(STATUS "\n-- UNIT TEST: ")
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${PROJECT_NAME})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(utest_src_file)



if( urdfdom_FOUND )
   set_property( GLOBAL
                 PROPERTY COMPILE_DEFINITIONS "HAVE_URDF_PARSE" )
endif( urdfdom_FOUND )
