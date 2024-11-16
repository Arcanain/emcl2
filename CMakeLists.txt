cmake_minimum_required(VERSION 3.8)
project(emcl2)

# C++の標準を設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# コンパイラオプションを設定
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージを自動で探す
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# 実行可能ファイルを作成
ament_auto_add_executable(emcl2_node
  src/Mcl.cpp
  src/ExpResetMcl2.cpp
  src/Particle.cpp
  src/OdomModel.cpp
  src/Pose.cpp
  src/Scan.cpp
  src/LikelihoodFieldMap.cpp
  src/emcl2_node.cpp
)

# テスト環境の設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# スクリプトファイルのインストール
install(PROGRAMS
  test/test.bash
  test/test_gui.bash
  DESTINATION lib/${PROJECT_NAME}
)

# ディレクトリごとのインストール設定
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

# ament_auto_packageでの共有設定
ament_auto_package(
  INSTALL_TO_SHARE
  config
  rviz
  launch
  map
  test
)
