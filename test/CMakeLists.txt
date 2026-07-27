cmake_minimum_required(VERSION 3.10)
project(line_follower_tests CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The two modules under test are compiled straight out of the firmware tree.
# They are the same files that go on the microcontroller, not copies: if they
# drift, these tests stop building.
set(FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/line_follower)

add_executable(run_tests
  test_main.cpp
  test_line_policy.cpp
  test_route.cpp
  ${FIRMWARE_DIR}/line_policy.cpp
  ${FIRMWARE_DIR}/route.cpp
)

target_include_directories(run_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${FIRMWARE_DIR}
)

target_compile_options(run_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

enable_testing()
add_test(NAME unit COMMAND run_tests)
