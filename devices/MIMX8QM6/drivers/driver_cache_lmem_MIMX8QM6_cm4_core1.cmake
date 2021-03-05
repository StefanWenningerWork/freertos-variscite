if(NOT DRIVER_CACHE_LMEM_MIMX8QM6_cm4_core1_INCLUDED)
    
    set(DRIVER_CACHE_LMEM_MIMX8QM6_cm4_core1_INCLUDED true CACHE BOOL "driver_cache_lmem component is included.")

    target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/fsl_cache.c
    )


    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/.
    )


    include(driver_common_MIMX8QM6_cm4_core1)

endif()
