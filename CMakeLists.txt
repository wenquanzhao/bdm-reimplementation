cmake_minimum_required(VERSION 3.16)
project(bdm_reimplementation VERSION 0.3 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(bdm STATIC
  src/boundary_map.cpp
  src/occupancy_grid.cpp
  src/local_grid.cpp
  src/incremental_mapper.cpp
  src/kitti_loader.cpp
  src/consistency.cpp)
target_include_directories(bdm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(bdm PRIVATE -Wall -Wextra -Wpedantic)
enable_testing()
add_executable(bdm_test tests/test_boundary_map.cpp)
target_link_libraries(bdm_test PRIVATE bdm)
target_compile_options(bdm_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME boundary_map COMMAND bdm_test)

add_executable(bdm_kitti_test tests/test_kitti_grid.cpp)
target_link_libraries(bdm_kitti_test PRIVATE bdm)
target_compile_options(bdm_kitti_test PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(bdm_kitti_test PRIVATE
  BDM_FIXTURE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/kitti_mini")
add_test(NAME kitti_grid COMMAND bdm_kitti_test)

add_executable(bdm_kitti src/kitti_demo.cpp)
target_link_libraries(bdm_kitti PRIVATE bdm)
target_compile_options(bdm_kitti PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME kitti_fixture
  COMMAND bdm_kitti
    --root ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/kitti_mini
    --sequence 00
    --resolution 1
    --min-range 0.1
    --max-range 30
    --padding 1)
set_tests_properties(kitti_fixture PROPERTIES PASS_REGULAR_EXPRESSION "mismatched: 0")
add_test(NAME kitti_help COMMAND bdm_kitti --help)
set_tests_properties(kitti_help PROPERTIES PASS_REGULAR_EXPRESSION "report Section 8")

add_executable(bdm_incremental_test tests/test_incremental.cpp)
target_link_libraries(bdm_incremental_test PRIVATE bdm)
target_compile_options(bdm_incremental_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME incremental_map COMMAND bdm_incremental_test)

add_executable(bdm_incremental src/incremental_demo.cpp)
target_link_libraries(bdm_incremental PRIVATE bdm)
target_compile_options(bdm_incremental PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME incremental_synthetic COMMAND bdm_incremental --synthetic)
set_tests_properties(incremental_synthetic PROPERTIES
  PASS_REGULAR_EXPRESSION "incremental_commit: yes")
add_test(NAME incremental_help COMMAND bdm_incremental --help)
set_tests_properties(incremental_help PROPERTIES
  PASS_REGULAR_EXPRESSION "not a Section 8 metric")
add_test(NAME incremental_kitti_mini
  COMMAND bdm_incremental
    --root ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/kitti_mini
    --sequence 00
    --resolution 1
    --min-range 0.1
    --max-range 30
    --size 16
    --slide-threshold 1
    --max-frames 2)
set_tests_properties(incremental_kitti_mini PROPERTIES
  PASS_REGULAR_EXPRESSION "overlap_mismatched: 0")
