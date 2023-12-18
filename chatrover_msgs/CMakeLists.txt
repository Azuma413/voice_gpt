cmake_minimum_required(VERSION 3.8)
project(chatrover_msgs)

find_package(ament_cmake_auto REQUIRED)
#xmlから依存を見つける
ament_auto_find_build_dependencies()

#自作msg/srvの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SendPos.srv"
  "srv/TextText.srv"
  "srv/ImageText.srv"
  "action/SendPos.action"
  #上のカスタムメッセージが依存しているパッケージを加える
  DEPENDENCIES
)

ament_auto_package()
