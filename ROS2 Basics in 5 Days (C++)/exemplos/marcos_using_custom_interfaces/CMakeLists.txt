cmake_minimum_required(VERSION 3.8)
project(marcos_using_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#-----Usando a interface em um nó-------------------
find_package(marcos_custom_interfaces REQUIRED) # This is the package that contains the custom interface
#---------------------------------------------------

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

#------para gerar o executável--------------------
add_executable(publish_age_node src/publish_age.cpp)
#------adiciona as dependencias ao nó--------------
ament_target_dependencies(publish_age_node rclcpp std_msgs marcos_custom_interfaces) # Note that we are also adding the package which contains the custom interface as a dependency of the node that will be using this custom interface
#---------------------------------------------------

#------ Arquivos de inicialização do nó---------
install(TARGETS
   publish_age_node
   DESTINATION lib/${PROJECT_NAME}
 )
#-----------------------------------------------

ament_package()
