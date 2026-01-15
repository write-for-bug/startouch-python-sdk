# ========================
# Startouch Python Binding
# ========================
cmake_minimum_required(VERSION 3.16)
project(startouch_python LANGUAGES CXX)

# --- C++ 标准 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 查找依赖 ---
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED CONFIG)
find_package(Threads REQUIRED)

# --- 定位原生库 ---
set(LIB_STARTOUCH ${CMAKE_CURRENT_SOURCE_DIR}/src/startouch_python_sdk/libstartouch.so)

if(NOT EXISTS ${LIB_STARTOUCH})
    message(FATAL_ERROR "libstartouch.so not found at ${LIB_STARTOUCH}")
endif()

# --- 构建 Python 扩展模块 ---
pybind11_add_module(startouch_python_sdk src/startouch_python_sdk/pybind_0825testfast.cpp)

# --- 包含路径 ---
target_include_directories(startouch_python_sdk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/startouch_python_sdk
    ${EIGEN3_INCLUDE_DIR}
)

# --- 链接库 ---
target_link_libraries(startouch_python_sdk PRIVATE
    ${LIB_STARTOUCH}
    Threads::Threads
    Eigen3::Eigen
)

# --- 设置 RPATH ---
set_target_properties(startouch_python_sdk PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_RPATH "$ORIGIN"
    SKIP_RPATH FALSE
)

# --- 安装规则：确保两个文件都安装到同一个目录 ---
install(TARGETS startouch_python_sdk
    LIBRARY DESTINATION startouch_python_sdk
    RUNTIME DESTINATION startouch_python_sdk
)

# 安装原生库到同一个目录
install(FILES ${LIB_STARTOUCH}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/startouch_python_sdk/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/src/startouch_python_sdk/startouchclass.py
    DESTINATION startouch_python_sdk
)

install(DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/startouch_python_sdk/param_csv_gripper/
    DESTINATION startouch_python_sdk/param_csv_gripper
    FILES_MATCHING
    PATTERN "*.csv"
    # PATTERN "*.txt" EXCLUDE  # 如果有其他不需要的文件
)