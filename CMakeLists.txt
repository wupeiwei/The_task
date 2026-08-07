cmake_minimum_required(VERSION 3.22)

# ============================================================
# 双板单工程（任务书要求：同一工程 + 条件编译区分）
#   用法：cmake --preset gimbal   （或 -DBOARD=gimbal）
#         cmake --preset chassis  （或 -DBOARD=chassis）
# 构建细节由 boards/<board>/CMakeLists.txt（CubeMX 生成）承接，
# 本文件仅做板型选择与路径修正，CubeMX 重新生成后无需改动。
# ============================================================

set(BOARD "gimbal" CACHE STRING "目标板型: gimbal / chassis")
if(NOT BOARD STREQUAL "gimbal" AND NOT BOARD STREQUAL "chassis")
    message(FATAL_ERROR "BOARD 必须为 gimbal 或 chassis（如 -DBOARD=gimbal）")
endif()

project(gimbal_dual C ASM)

# 修正工具链文件基于 CMAKE_SOURCE_DIR 注入的链接脚本路径
# （子目录场景下 CMAKE_SOURCE_DIR 为仓库根，需指向本板目录；
#   必须在 add_subdirectory 之前，子目录 target 会继承本值）
string(REGEX REPLACE "-T \"[^\"]*STM32F103XX_FLASH\\.ld\""
       "-T \"${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}/STM32F103XX_FLASH.ld\""
       CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

add_subdirectory(boards/${BOARD})
