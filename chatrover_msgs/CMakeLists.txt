cmake_minimum_required(VERSION 3.8)
project(chatrover_msgs)

find_package(ament_cmake_auto REQUIRED)
#xmlから依存を見つける
ament_auto_find_build_dependencies()

#自作msg/srvの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ObjectPosition.srv"
  "srv/TextText.srv"
  #上のカスタムメッセージが依存しているパッケージを加える
  DEPENDENCIES geometry_msgs
)

ament_auto_package()
