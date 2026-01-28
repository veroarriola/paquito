cmake_minimum_required(VERSION 3.8)
project(paquito_raspberry_cpp)
# Autora: AlevAriasV

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. ENCONTRAR PAQUETES (Solo los necesarios)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. DEFINICIÓN DEL EJECUTABLE PWMConverter
# Asegúrate de que tu archivo esté en la carpeta 'src'
add_executable(command_executor src/PWMConverter.cpp)

#publisher
add_executable(car_velocity_subscriber src/car_velocity_subscriber.cpp)
target_include_directories(car_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(car_subscriber rclcpp geometry_msgs std_msgs)


target_include_directories(command_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(command_executor PUBLIC c_std_99 cxx_std_17)

# Dependencias del nodo
ament_target_dependencies(command_executor 
  rclcpp 
  std_msgs
)

# 3. INSTALACIÓN
install(TARGETS
  command_executor
  car_velocity_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

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
