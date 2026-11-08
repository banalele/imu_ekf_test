cmake_minimum_required(VERSION 3.20)

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

# 把同目录下的 algo 注册为 Zephyr 模块（algo/zephyr/module.yml）
# 注意：必须在 find_package(Zephyr) 之前
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/algo)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(breeze_template)

# 直接添加所有源文件到 app
target_sources(app PRIVATE

    src2/main.cpp
    src2/imu_xrobot.c
    src2/imu_uart.c
    src2/rtt.c


)
# 添加头文件路径
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/algo/include   # algo 算法库头文件（robotpilots/algorithm/*.hpp）
    src2
)

