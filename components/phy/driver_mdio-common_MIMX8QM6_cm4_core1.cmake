if(NOT DRIVER_MDIO-COMMON_MIMX8QM6_cm4_core1_INCLUDED)
    
    set(DRIVER_MDIO-COMMON_MIMX8QM6_cm4_core1_INCLUDED true CACHE BOOL "driver_mdio-common component is included.")


    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/.
    )

    include(driver_common_MIMX8QM6_cm4_core1)

endif()
