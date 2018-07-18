###Cmake学习教程 
* 学习不错的网站：
    * [官方学习教程](https://cmake.org/cmake-tutorial/)
    * [官方帮助文档](https://cmake.org/cmake/help/latest/)
* 照着官方文档走一遍Cmake学习教程
* cmake是一个跨平台的编译（安装）工具，可以在linux下生成makefile，在Windows下生成project。方便代码的跨平台移植，在linux中避免了makefile的编写。
####Cmake的语法规则
* cmake的指令大小写都可以使用, 例如SET()可以写成set()
* 变量有大小写之分, 例如SRC_LIST与src_list不同
* 变量使用方式, 在取值时采用${SRC_LIST}取值, 但在IF语句中直接使用变量名
* 参数使用空格或者分号(;)分隔, 也可以使用括号()扩起
####外部构建
* 将构建过程中的中间临时文件与文件分离，在源码文件夹下单独创建一个放置生成工程的空文件夹，例如build，在build目录中运行`cmake ..`命令即可。其中".."表示父目录
* 目录变量
    * 该变量为定义好的变量, 不需要再定义, 变量PROJECT_BINARY_DIR表示生成的工程文件的地址
    * 变量PROJECT_SOURCE_DIR表示源文件的地址
* 定义变量
    ```
    set(SRC_LIST idealrobot.hpp robot.hpp robot.cpp gamepad.hpp main.cpp)
    ```
将SRC_LIST定义为后面的文字
* 注释用井号键`#`
* 指定版本号 `cmake_minimum_required(VERSION 2.4)` 最小支持的版本号
* 用project指定项目名称 `project(haha)`
* 利用set设置变量
* 利用`include_directories`来添加头文件的搜索路径`include_directories(./include)` ---头文件在目录为CMakeLists所在文件夹下的include文件夹下。
* 利用`link_directories`添加动态链接库或者静态链接库的搜索路径`link_directories(/usr/lib)`---添加的路径为/usr/lib文件夹
* 生成工程, 利用add_executable(telerobot ${SRC_LIST}) --- 将SRC_LIST变量所代表的文件进行编译，生成工程。
* 指令链接的动态库或静态库`target_link_libraries(telerobot libAria.so pthread)`---只能链接该指令之前`link_directories()`路径中的库。
* 编译选项设置 `target_compile_options(telerobot PRIVATE -std=c++11) `-----f必须在目标工程已经存在的情况下使用，即add_executable() 或 add_library()之后。

* 简单模板
```
cmake_minimum_required(VERSION 2.4)   #设置编译版本的环境
project(telerobot)                    #工程名字

set(SRC_LIST idealrobot.hpp robot.hpp robot.cpp gamepad.hpp
    main.cpp)                         #设置SRC_LIST变量
#设置CMAKE_CXX_FLAGS_DEBUG 和 CMAKE_CXX_FLAGS_RELEASE变量
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb ")
SET(CMAKE_CXX_FLAGS_RELEASE "${ENV{CXXFLAGS} -O3 -Wall")

include_directories(./include)          #包含头文件       
link_directories(/usr/lib)              #链接文件目录 
#目录变量
MESSAGE(STATUS "This is BINARY dir " ${PROJECT_BINARY_DIR}) 
MESSAGE(STATUS "This is SOURCE dir "${PROJECT_SOURCE_DIR}) 

add_executable(telerobot ${SRC_LIST})   #生成工程
# 链接动态库
target_link_libraries(telerobot libAria.so pthread) 
target_compile_options(telerobot PRIVATE -std=c++11) 
#编译选项设置, 用c++11来编译
set(CMAKE_CONFIGURATION_TYPES "Release")    #设置变量
```
