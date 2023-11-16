cmake_minimum_required(VERSION 3.8)
project(chat_rover_bt)

find_package(ament_cmake_auto REQUIRED)

#xmlから依存を見つける
ament_auto_find_build_dependencies()

#実行ファイルの生成
ament_auto_add_executable(
main_node src/main_node.cpp include/my_action_node.hpp include/my_ros_node.hpp
)
ament_auto_add_executable(
  action_client src/action_client.cpp
)
ament_auto_add_executable(
  action_server src/action_server.cpp
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#構文チェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#ライブラリのビルド
ament_auto_package()