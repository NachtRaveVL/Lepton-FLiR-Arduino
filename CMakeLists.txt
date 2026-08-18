cmake_minimum_required(VERSION 3.16)
project(LeptonFLiRHostTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB LEPTONFLIR_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_library(leptonflir_host STATIC ${LEPTONFLIR_SOURCES} tests/arduino/arduino_stubs.cpp)
target_include_directories(leptonflir_host PUBLIC src tests/arduino)
target_compile_definitions(leptonflir_host PUBLIC ARDUINO=10819)
target_compile_options(leptonflir_host PRIVATE -Wall -Wextra -Wpedantic)

add_library(leptonflir_compile_debug OBJECT ${LEPTONFLIR_SOURCES})
target_include_directories(leptonflir_compile_debug PRIVATE src tests/arduino)
target_compile_definitions(leptonflir_compile_debug PRIVATE ARDUINO=10819 LEPFLIR_ENABLE_DEBUG_OUTPUT)
target_compile_options(leptonflir_compile_debug PRIVATE -Wall -Wextra -Wpedantic)

add_library(leptonflir_compile_soft_i2c OBJECT ${LEPTONFLIR_SOURCES})
target_include_directories(leptonflir_compile_soft_i2c PRIVATE src tests/arduino)
target_compile_definitions(leptonflir_compile_soft_i2c PRIVATE ARDUINO=10819 LEPFLIR_ENABLE_SOFTWARE_I2C)
target_compile_options(leptonflir_compile_soft_i2c PRIVATE -Wall -Wextra -Wpedantic)


add_library(leptonflir_dma_host STATIC ${LEPTONFLIR_SOURCES} tests/arduino/arduino_stubs.cpp)
target_include_directories(leptonflir_dma_host PUBLIC src tests/arduino)
target_compile_definitions(leptonflir_dma_host PUBLIC ARDUINO=10819 SPI_HAS_TRANSFER_ASYNC)
target_compile_options(leptonflir_dma_host PRIVATE -Wall -Wextra -Wpedantic)

add_executable(leptonflir_dma_tests tests/dma_tests.cpp)
target_link_libraries(leptonflir_dma_tests PRIVATE leptonflir_dma_host)

add_executable(leptonflir_tests tests/test_main.cpp)
target_link_libraries(leptonflir_tests PRIVATE leptonflir_host)

add_executable(leptonflir_module_tests tests/module_tests.cpp)
target_link_libraries(leptonflir_module_tests PRIVATE leptonflir_host)

enable_testing()
add_test(NAME leptonflir_tests COMMAND leptonflir_tests)
add_test(NAME leptonflir_module_tests COMMAND leptonflir_module_tests)
add_test(NAME leptonflir_dma_tests COMMAND leptonflir_dma_tests)


function(add_lepton_example_compile target source)
    add_library(${target} OBJECT ${source})
    target_include_directories(${target} PRIVATE src tests/arduino)
    target_compile_definitions(${target} PRIVATE ARDUINO=10819)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

add_lepton_example_compile(leptonflir_example_simple tests/examples/simple.cpp)
add_lepton_example_compile(leptonflir_example_advanced tests/examples/advanced.cpp)
add_lepton_example_compile(leptonflir_example_image_capture tests/examples/image_capture.cpp)
add_lepton_example_compile(leptonflir_example_module_info tests/examples/module_info.cpp)
target_compile_definitions(leptonflir_example_module_info PRIVATE LEPFLIR_ENABLE_DEBUG_OUTPUT)
add_lepton_example_compile(leptonflir_example_software_i2c tests/examples/software_i2c.cpp)
target_compile_definitions(leptonflir_example_software_i2c PRIVATE LEPFLIR_ENABLE_SOFTWARE_I2C)
add_lepton_example_compile(leptonflir_example_dma_capture tests/examples/dma_capture.cpp)
