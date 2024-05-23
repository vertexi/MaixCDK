############### Add include ###################
list(APPEND ADD_INCLUDE "include"
    )
list(APPEND ADD_PRIVATE_INCLUDE "")
###############################################

############ Add source files #################
# list(APPEND ADD_SRCS  "src/main.c"
#                       "src/test.c"
#     )

set(middleware_version_str "${CONFIG_SOPHGO_MIDDLEWARE_VERSION_MAJOR}.${CONFIG_SOPHGO_MIDDLEWARE_VERSION_MINOR}.${CONFIG_SOPHGO_MIDDLEWARE_VERSION_PATCH}")
set(middleware_unzip_path "${DL_EXTRACTED_PATH}/sophgo-middleware")
set(middleware_src_path "${middleware_unzip_path}/sophgo-middleware-${middleware_version_str}")

list(APPEND ADD_REQUIREMENTS cvi_tpu)
list(APPEND ADD_REQUIREMENTS atomic)
set(middleware_include_dir .
        ${middleware_src_path}/v2/component/panel/${CONFIG_SOPHGO_MIDDLEWARE_CHIP}
        ${middleware_src_path}/v2/include
        ${middleware_src_path}/v2/include/isp/${CONFIG_SOPHGO_MIDDLEWARE_CHIP}
        ${middleware_src_path}/v2/sample/common
        ${middleware_src_path}/v2/uapi
        ${middleware_src_path}/v2/3rdparty/inih
        ${middleware_src_path}/v2/modules/ive/include/
    )
list(APPEND ADD_INCLUDE ${middleware_include_dir})
set_property(SOURCE ${middleware_include_dir} PROPERTY GENERATED 1)

append_srcs_dir(middleware_src_dir  ${middleware_src_path}/v2/sample/common
                                ${middleware_src_path}/v2/component/isp/sensor/cv181x/gcore_gc4653
                                )
list(APPEND ADD_SRCS        ${middleware_src_dir})
set_property(SOURCE ${middleware_src_dir} PROPERTY GENERATED 1)

set(middleware_static_lib_file  "")
list(APPEND ADD_STATIC_LIB ${middleware_static_lib_file})
set_property(SOURCE ${middleware_static_lib_file} PROPERTY GENERATED 1)

if(${middleware_version_str} VERSION_EQUAL "0.0.4")
set(mmf_lib_dir ${middleware_src_path}/v2/lib)
else()
set(mmf_lib_dir ${middleware_src_path}/v2/${CONFIG_SOPHGO_MIDDLEWARE_CHIP}/lib_${CONFIG_SOPHGO_MIDDLEWARE_C_LIBRARY}_riscv64)
endif()
if(CONFIG_SOPHGO_MIDDLEWARE_C_LIBRARY STREQUAL "musl")
if(${middleware_version_str} VERSION_EQUAL "0.0.4")
    set(middleware_dynamic_lib_file     ${mmf_lib_dir}/libcvi_ive.so
                                        ${mmf_lib_dir}/libcvi_bin.so
                                        ${mmf_lib_dir}/libaaccomm2.so
                                        ${mmf_lib_dir}/libaacdec2.so
                                        ${mmf_lib_dir}/libaacenc2.so
                                        ${mmf_lib_dir}/libaacsbrdec2.so
                                        ${mmf_lib_dir}/libaacsbrenc2.so
                                        ${mmf_lib_dir}/libae.so
                                        ${mmf_lib_dir}/libaf.so
                                        ${mmf_lib_dir}/libawb.so
                                        ${mmf_lib_dir}/libcvi_audio.so
                                        ${mmf_lib_dir}/libcvi_bin_isp.so
                                        ${mmf_lib_dir}/libcvi_bin.so
                                        ${mmf_lib_dir}/libcvi_ispd2.so
                                        ${mmf_lib_dir}/libcvi_RES1.so
                                        ${mmf_lib_dir}/libcvi_ssp.so
                                        ${mmf_lib_dir}/libcvi_VoiceEngine.so
                                        ${mmf_lib_dir}/libcvi_vqe.so
                                        ${mmf_lib_dir}/libdnvqe.so
                                        ${mmf_lib_dir}/libisp_algo.so
                                        ${mmf_lib_dir}/libisp.so
                                        ${mmf_lib_dir}/libmipi_tx.so
                                        ${mmf_lib_dir}/libmisc.so
                                        ${mmf_lib_dir}/libosdc.so
                                        ${mmf_lib_dir}/libraw_dump.so
                                        ${mmf_lib_dir}/libsys.so
                                        ${mmf_lib_dir}/libvdec.so
                                        ${mmf_lib_dir}/libvenc.so
                                        ${mmf_lib_dir}/libvpu.so
                                        ${mmf_lib_dir}/libjson-c.so.5
                                        ${mmf_lib_dir}/libtinyalsa.so
                                        ${mmf_lib_dir}/3rd/libcli.so
                                        ${mmf_lib_dir}/3rd/libini.so)
else()
    set(middleware_dynamic_lib_file     ${mmf_lib_dir}/libcvi_ive.so
                                        ${mmf_lib_dir}/libcvi_bin.so
                                        ${mmf_lib_dir}/libaaccomm2.so
                                        ${mmf_lib_dir}/libaacdec2.so
                                        ${mmf_lib_dir}/libaacenc2.so
                                        ${mmf_lib_dir}/libaacsbrdec2.so
                                        ${mmf_lib_dir}/libaacsbrenc2.so
                                        ${mmf_lib_dir}/libae.so
                                        ${mmf_lib_dir}/libaf.so
                                        ${mmf_lib_dir}/libawb.so
                                        ${mmf_lib_dir}/libcipher.so
                                        ${mmf_lib_dir}/libcvi_audio.so
                                        ${mmf_lib_dir}/libcvi_bin_isp.so
                                        ${mmf_lib_dir}/libcvi_bin.so
                                        ${mmf_lib_dir}/libcvi_ispd2.so
                                        ${mmf_lib_dir}/libcvi_RES1.so
                                        ${mmf_lib_dir}/libcvi_ssp.so
                                        ${mmf_lib_dir}/libcvi_VoiceEngine.so
                                        ${mmf_lib_dir}/libcvi_vqe.so
                                        ${mmf_lib_dir}/libdnvqe.so
                                        ${mmf_lib_dir}/libisp_algo.so
                                        ${mmf_lib_dir}/libisp.so
                                        ${mmf_lib_dir}/libmipi_tx.so
                                        ${mmf_lib_dir}/libmisc.so
                                        ${mmf_lib_dir}/libosdc.so
                                        ${mmf_lib_dir}/libraw_dump.so
                                        # ${mmf_lib_dir}/libraw_replay.so
                                        # ${mmf_lib_dir}/libsns_full.so
                                        # ${mmf_lib_dir}/libsns_gc4653.so
                                        ${mmf_lib_dir}/libsys.so
                                        ${mmf_lib_dir}/libvdec.so
                                        ${mmf_lib_dir}/libvenc.so
                                        ${mmf_lib_dir}/libvpu.so
                                        ${mmf_lib_dir}/libz.so
                                        ${mmf_lib_dir}/libtinyalsa.so
                                        ${mmf_lib_dir}/3rd/libcli.so
                                        ${mmf_lib_dir}/3rd/libini.so
                                        ${mmf_lib_dir}/3rd/libjson-c.so.5)
endif()
elseif(CONFIG_SOPHGO_MIDDLEWARE_C_LIBRARY STREQUAL "glibc")
set(middleware_dynamic_lib_file     ${mmf_lib_dir}/libcvi_vb.so
                                    ${mmf_lib_dir}/libcvi_sys.so
                                    ${mmf_lib_dir}/libcvi_debug.so
)
else()
message(FATAL_ERROR "No sophgo middleware c library support, please check menuoconfig to select c library")
endif()
list(APPEND ADD_DYNAMIC_LIB ${middleware_dynamic_lib_file})
set_property(SOURCE ${middleware_dynamic_lib_file} PROPERTY GENERATED 1)

FILE(GLOB EXTRA_DYNAMIC_LIB  "lib/*.so")
list(APPEND ADD_DYNAMIC_LIB ${EXTRA_DYNAMIC_LIB})
set_property(SOURCE ${EXTRA_DYNAMIC_LIB} PROPERTY GENERATED 1)

list(APPEND ADD_DEFINITIONS_PRIVATE -DSENSOR_GCORE_GC4653
                            -DSENSOR0_TYPE=SONY_IMX327_MIPI_2M_30FPS_12BIT
                            -DSENSOR1_TYPE=SONY_IMX327_MIPI_2M_30FPS_12BIT)

append_srcs_dir(ADD_SRCS "src")       # append source file in src dir to var ADD_SRCS
# list(REMOVE_ITEM COMPONENT_SRCS "src/test2.c")
# FILE(GLOB_RECURSE EXTRA_SRC  "src/*.c")
# FILE(GLOB EXTRA_SRC  "src/*.c")
# list(APPEND ADD_SRCS  ${EXTRA_SRC})
# aux_source_directory(src ADD_SRCS)  # collect all source file in src dir, will set var ADD_SRCS
# append_srcs_dir(ADD_SRCS "src")     # append source file in src dir to var ADD_SRCS
# list(REMOVE_ITEM COMPONENT_SRCS "src/test.c")
# set(ADD_ASM_SRCS "src/asm.S")
# list(APPEND ADD_SRCS ${ADD_ASM_SRCS})
# SET_PROPERTY(SOURCE ${ADD_ASM_SRCS} PROPERTY LANGUAGE C) # set .S  ASM file as C language
# SET_SOURCE_FILES_PROPERTIES(${ADD_ASM_SRCS} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp -D BBBBB")
###############################################

###### Add required/dependent components ######
list(APPEND ADD_REQUIREMENTS basic)
list(APPEND ADD_REQUIREMENTS vision)
###############################################

###### Add link search path for requirements/libs ######
# list(APPEND ADD_LINK_SEARCH_PATH "${CONFIG_TOOLCHAIN_PATH}/lib")
# list(APPEND ADD_REQUIREMENTS pthread m)  # add system libs, pthread and math lib for example here
# set (OpenCV_DIR opencv/lib/cmake/opencv4)
# find_package(OpenCV REQUIRED)
###############################################

############ Add static libs ##################
# list(APPEND ADD_STATIC_LIB "lib/libtest.a")
###############################################

#### Add compile option for this component ####
#### Just for this component, won't affect other 
#### modules, including component that depend 
#### on this component
# list(APPEND ADD_DEFINITIONS_PRIVATE -DAAAAA=1)

#### Add compile option for this component
#### and components denpend on this component
# list(APPEND ADD_DEFINITIONS -DAAAAA222=1
#                             -DAAAAA333=1)
###############################################

############ Add static libs ##################
#### Update parent's variables like CMAKE_C_LINK_FLAGS
# set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group libmaix/libtest.a -ltest2 -Wl,--end-group" PARENT_SCOPE)
###############################################

######### Add files need to download #########
# list(APPEND ADD_FILE_DOWNLOADS "{
#     'url': 'https://*****/abcde.tar.xz',
#     'urls': [],  # backup urls, if url failed, will try urls
#     'sites': [], # download site, user can manually download file and put it into dl_path
#     'sha256sum': '',
#     'filename': 'abcde.tar.xz',
#     'path': 'toolchains/xxxxx',
#     }"
# )
#
# then extracted file in ${DL_EXTRACTED_PATH}/toolchains/xxxxx,
# you can directly use then, for example use it in add_custom_command
##############################################

# register component, DYNAMIC or SHARED flags will make component compiled to dynamic(shared) lib
register_component()
