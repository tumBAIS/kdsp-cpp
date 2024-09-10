CPMAddPackage("gh:catchorg/Catch2@2.13.8")
set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE CATCH_SOURCES "${PROJECT_SOURCE_DIR}/catch2_tests/*.cpp")
add_executable(catch2_tests ${CATCH_SOURCES})
target_include_directories(catch2_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(catch2_tests PUBLIC ${Catch2_SOURCE_DIR})
target_link_libraries(catch2_tests PRIVATE Catch2)