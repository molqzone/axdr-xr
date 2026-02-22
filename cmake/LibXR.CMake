set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# LibXR
set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER st)
set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
set(XRFOC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/XRFOC)
set(XRMATH_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/XRMath)
set(LIBRAT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/librat)
set(CMSIS_DSP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/CMSIS-DSP)
set(XRFOC_INCLUDE_DIRS
    ${XRFOC_ROOT}/src
    ${XRFOC_ROOT}/src/driver/foc
    ${XRFOC_ROOT}/driver)
set(XRMATH_INCLUDE_DIRS
    ${XRMATH_ROOT}/src
    ${XRMATH_ROOT}/src/driver
    ${XRMATH_ROOT}/driver
    ${CMSIS_DSP_ROOT}/PrivateInclude
    ${CMSIS_DSP_ROOT}/Include)
set(XRMATH_CMSIS_DSP_C_SOURCES
    ${CMSIS_DSP_ROOT}/Source/CommonTables/arm_common_tables.c
    ${CMSIS_DSP_ROOT}/Source/ControllerFunctions/arm_sin_cos_f32.c
    ${CMSIS_DSP_ROOT}/Source/FastMathFunctions/arm_sin_f32.c
    ${CMSIS_DSP_ROOT}/Source/FastMathFunctions/arm_cos_f32.c
    ${CMSIS_DSP_ROOT}/Source/FastMathFunctions/arm_atan2_f32.c)

add_subdirectory(Middlewares/Third_Party/LibXR)
add_subdirectory(${LIBRAT_ROOT} ${CMAKE_BINARY_DIR}/librat)
target_include_directories(librat PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)
target_link_libraries(xr
    PUBLIC stm32cubemx
)

target_include_directories(xr
    PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>
    PUBLIC Core/Inc
    PUBLIC User
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
    PUBLIC User
    PUBLIC ${XRFOC_INCLUDE_DIRS}
    PUBLIC ${XRMATH_INCLUDE_DIRS}
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    xr
    librat
)

file(
    GLOB LIBXR_USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp")


target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE ${LIBXR_USER_SOURCES}
    PRIVATE ${XRMATH_CMSIS_DSP_C_SOURCES}
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ARM_MATH_CM4)

set(XRFOC_HOT_CXX "${CMAKE_CURRENT_SOURCE_DIR}/User/app_main.cpp")
if(EXISTS "${XRFOC_HOT_CXX}")
    set_source_files_properties("${XRFOC_HOT_CXX}" PROPERTIES
        COMPILE_OPTIONS "-O3;-ffast-math;-fno-math-errno")
endif()

set(XRFOC_ISR_C "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32g4xx_it.c")
if(EXISTS "${XRFOC_ISR_C}")
    set_source_files_properties("${XRFOC_ISR_C}" PROPERTIES
        COMPILE_OPTIONS "-O3")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Og)
    target_compile_options(xr PRIVATE -O2)
    if(TARGET FreeRTOS)
        target_compile_options(FreeRTOS PRIVATE -O2)
    endif()

    if(TARGET STM32_Drivers)
        target_compile_options(STM32_Drivers PRIVATE -O2)
    endif()

    if(TARGET USB_Device_Library)
        target_compile_options(USB_Device_Library PRIVATE -O2)
    endif()
endif()
