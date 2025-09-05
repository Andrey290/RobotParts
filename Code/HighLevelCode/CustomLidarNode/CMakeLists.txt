cmake_minimum_required(VERSION 3.12)
project(CustomLidarNode)

# Настройка стандарта C
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Поиск библиотеки pigpio
find_library(PIGPIO_LIB pigpio)
if(NOT PIGPIO_LIB)
    message(FATAL_ERROR "pigpio library not found")
endif()

# Добавление исполняемого файла
add_executable(CustomLidarNode
    src/main.c
    src/parser.c
    src/gpio.c
    src/serial.c
)

# Директории с заголовками
target_include_directories(CustomLidarNode PRIVATE inc)

# Линковка с математической библиотекой
target_link_libraries(CustomLidarNode m)

# Флаги компиляции
target_compile_options(CustomLidarNode PRIVATE -Wall -Wextra)

# Библиотека пинов
target_link_libraries(CustomLidarNode m ${PIGPIO_LIB} pthread)
