cmake_minimum_required(VERSION 3.8)
project(ltme_node)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ltme_interfaces REQUIRED)
find_package(OpenSSL)

###########
## Test ##
###########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###########
## Build ##
###########

# ldcp_sdk lib
set(LDCP_SDK_DIR "lib/ldcp_sdk")
add_subdirectory(${LDCP_SDK_DIR})

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${LDCP_SDK_DIR}/include"
)

add_executable(ltme_node src/ltme_node.cpp)
ament_target_dependencies(ltme_node rclcpp sensor_msgs std_msgs std_srvs ltme_interfaces)
target_link_libraries(ltme_node ldcp_sdk)

if(OPENSSL_FOUND)
  add_executable(update_firmware
    "src/update_firmware.cpp"
  )

  target_link_libraries(update_firmware
    ldcp_sdk
    OpenSSL::Crypto
  )
endif()

#############
## Install ##
#############

# install nodes
install(TARGETS
  ltme_node update_firmware
  DESTINATION lib/${PROJECT_NAME}
)

# install launch directory
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()