#!/bin/bash
SCRIPT_PATH=$(readlink -f "$BASH_SOURCE")
echo $SCRIPT_PATH
cd "$(dirname "$SCRIPT_PATH")"
# 检查CMakeListsROS2.txt是否存在
if [ -f "CMakeListsROS2.txt" ]; then
    # 将CMakeListsROS2.txt复制到CMakeLists.txt
    cp "CMakeListsROS2.txt" "CMakeLists.txt"
    echo "CMakeListsROS2.txt has been copy to CMakeLists.txt"
else
    echo "CMakeListsROS2.txt does not exist in the current directory."
fi

# 检查当前路径下是否存在package_ROS.xml文件
if [ -f "package_ROS2.xml" ]; then
    # package_ROS2.xml复制到package.xml
    cp "package_ROS2.xml" "package.xml"
    echo "package_ROS2.xml has been copy to package.xml"
else
    echo "package_ROS2.xml does not exist in the current directory."
fi


