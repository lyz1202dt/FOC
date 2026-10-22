# STM32G431CB: 128 KiB Flash, with the last two 2 KiB pages reserved for config.
set(BOOT_FLASH_ORIGIN 0x08000000)
set(BOOT_FLASH_LENGTH 24576)
set(APP_FLASH_ORIGIN 0x08006000)
set(APP_FLASH_LENGTH 102400)
math(EXPR APP_VECTOR_OFFSET "${APP_FLASH_ORIGIN} - ${BOOT_FLASH_ORIGIN}" OUTPUT_FORMAT HEXADECIMAL)

function(configure_firmware target linker_script flash_origin flash_length)
    target_link_options(${target} PRIVATE
        "-T${linker_script}"
        "-Wl,--defsym=FIRMWARE_FLASH_ORIGIN=${flash_origin}"
        "-Wl,--defsym=FIRMWARE_FLASH_LENGTH=${flash_length}"
        "-Wl,-Map=${CMAKE_BINARY_DIR}/${target}.map")
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${linker_script}")
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}>
            "${CMAKE_BINARY_DIR}/${target}.bin"
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}>
            "${CMAKE_BINARY_DIR}/${target}.hex"
        BYPRODUCTS "${CMAKE_BINARY_DIR}/${target}.bin"
            "${CMAKE_BINARY_DIR}/${target}.hex"
        VERBATIM)
    set_property(TARGET ${target} APPEND PROPERTY ADDITIONAL_CLEAN_FILES
        "${CMAKE_BINARY_DIR}/${target}.map")
endfunction()
