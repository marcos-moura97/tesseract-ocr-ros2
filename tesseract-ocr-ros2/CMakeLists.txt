cmake_minimum_required(VERSION 3.8)
project(tesseract-ocr-ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "RELEASE")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Tesseract 5 REQUIRED)
if (NOT Tesseract_FOUND)
  find_package(PkgConfig REQUIRED)

  pkg_check_modules(Tesseract REQUIRED IMPORTED_TARGET tesseract lept)
  add_library(Tesseract::libtesseract ALIAS PkgConfig::Tesseract)
endif ()

add_executable(tesseract_ocr_node src/tesseract_ocr_node.cpp)
target_include_directories(tesseract_ocr_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Tesseract_INCLUDE_DIRS} 
  )
target_compile_features(tesseract_ocr_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  tesseract_ocr_node
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "rclcpp"
  OpenCV
)

target_link_libraries(tesseract_ocr_node ${Tesseract_LIBRARIES})  # you can link here multiple libraries as well.

install(TARGETS tesseract_ocr_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
