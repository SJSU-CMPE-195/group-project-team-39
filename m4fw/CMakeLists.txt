cmake_minimum_required(VERSION 3.16)

project(gantry_fw
    LANGUAGES C CXX ASM
    VERSION   0.2.0
)

# ---------- Target MCU flags ----------
set(CPU_FLAGS
    -mcpu=cortex-m4
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -mthumb
)

# ---------- Linker script ----------
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/device/gcc/MIMX8QM6xxxFF_cm4_core0_ram.ld")

# ---------- Source files ----------
set(APP_SOURCES
    app/main.cpp
    app/iSV57T.cpp
    app/limitSwitch.cpp
    app/gantry.cpp
    app/empty_rsc_table.c
)

set(BOARD_SOURCES
    board/board.c
    board/pin_mux.c
    board/clock_config.c
)

set(DEVICE_SOURCES
    device/system_MIMX8QM6_cm4_core0.c
    device/gcc/startup_MIMX8QM6_cm4_core0.S
)

# SDK peripheral drivers
file(GLOB DRIVER_SOURCES drivers/*.c)

# SCFW API RPC client sources (SCU IPC communication)
file(GLOB SCFW_SOURCES
    device/scfw_api/main/ipc_imx8qm.c
    device/scfw_api/svc/pm/pm_rpc_clnt.c
    device/scfw_api/svc/pad/pad_rpc_clnt.c
    device/scfw_api/svc/misc/misc_rpc_clnt.c
    device/scfw_api/svc/irq/irq_rpc_clnt.c
    device/scfw_api/svc/timer/timer_rpc_clnt.c
    device/scfw_api/svc/seco/seco_rpc_clnt.c
    device/scfw_api/svc/rm/rm_rpc_clnt.c
)

# FreeRTOS kernel sources
file(GLOB FREERTOS_CORE_SOURCES
    freertos/Source/tasks.c
    freertos/Source/queue.c
    freertos/Source/list.c
    freertos/Source/timers.c
    freertos/Source/event_groups.c
    freertos/Source/stream_buffer.c
)

set(FREERTOS_PORT_SOURCES
    freertos/Source/portable/GCC/ARM_CM4F/port.c
    freertos/Source/portable/MemMang/heap_4.c
)

# ---------- Executable ----------
add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${BOARD_SOURCES}
    ${DEVICE_SOURCES}
    ${DRIVER_SOURCES}
    ${SCFW_SOURCES}
    ${FREERTOS_CORE_SOURCES}
    ${FREERTOS_PORT_SOURCES}
)

# ---------- Include paths ----------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/board
    ${CMAKE_SOURCE_DIR}/device
    ${CMAKE_SOURCE_DIR}/device/scfw_api
    ${CMAKE_SOURCE_DIR}/drivers
    ${CMAKE_SOURCE_DIR}/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/freertos/Source/include
    ${CMAKE_SOURCE_DIR}/freertos/Source/portable/GCC/ARM_CM4F
)

# ---------- Compile definitions ----------
target_compile_definitions(${PROJECT_NAME} PRIVATE
    CPU_MIMX8QM6AVUFF_cm4_core0
    MIMX8QM6_cm4_core0
    MIMX8QM_CM4_CORE0
    __USE_CMSIS
)

# ---------- Compile options (C) ----------
set_source_files_properties(${BOARD_SOURCES} ${DEVICE_SOURCES} ${DRIVER_SOURCES}
    ${SCFW_SOURCES} ${FREERTOS_CORE_SOURCES} ${FREERTOS_PORT_SOURCES}
    app/empty_rsc_table.c
    PROPERTIES COMPILE_OPTIONS "-std=gnu11"
)

# ---------- Compile options (common) ----------
target_compile_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    -ffunction-sections
    -fdata-sections
    -ffreestanding
    -fno-builtin
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-Os>
)

# ---------- Compile options (C++) ----------
set_source_files_properties(
    app/main.cpp app/iSV57T.cpp app/limitSwitch.cpp app/gantry.cpp
    PROPERTIES COMPILE_OPTIONS "-std=c++17;-fno-exceptions;-fno-rtti;-fno-threadsafe-statics"
)

# Use the C compiler for linking (we don't need libstdc++)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

# ---------- Link options ----------
target_link_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-Map=${PROJECT_NAME}.map
)

# ---------- Post-build: generate .bin from .elf ----------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating ${PROJECT_NAME}.bin and printing size"
)
