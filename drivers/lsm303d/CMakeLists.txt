include(../../cmake/df_common.cmake)

include_directories(
        ../../framework/include
)

get_filename_component(drivername ${CMAKE_CURRENT_SOURCE_DIR} NAME)
df_add_library(df_${drivername}
        LSM303D.cpp
        )

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
