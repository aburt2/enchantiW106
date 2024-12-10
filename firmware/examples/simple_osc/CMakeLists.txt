# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.23.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(simple_osc)

FILE(GLOB app_sources src/*.c*)
FILE(GLOB liblo_sources 
    src/libmapper-nxp/src/lo/*.c
    src/libmapper-nxp/src/compat/*.c
    src/libmapper-nxp/src/zlib/*.c
)


target_sources(app PRIVATE 
    ${app_sources}
    ${liblo_sources}
)

# add_executable(compat 
#     gai_strerror.c
#     gethostname.c
#     getnameinfo.c
#     ifaddrs.c
# )

# add_executable(libmapper
#     expression/expr_buffer.h
#     expression/expr_constant.h
#     expression/expr_evaluator.h
#     expression/expr_function.h
#     expression/expr_lexer.h
#     expression/expr_operator.h
#     expression/expr_parser.h
#     expression/expr_stack.h
#     expression/expr_struct.h
#     expression/expr_token.h
#     expression/expr_trace.h
#     expression/expr_value.h
#     expression/expr_variable.h
#     util/mpr_debug.h
#     util/mpr_inline.h
#     util/mpr_set_coerced.c
#     util/mpr_set_coerced.h
#     bitflags.h
#     config.h
#     device.c
#     device.h
#     expression.c
#     expression.h
#     graph.c
#     graph.h
#     id.h
#     id_map.h
#     link.c
#     link.h
#     list.c
#     list.h
#     map.c
#     map.h
#     mapper.h
#     mapper_constants.h
#     mapper_cpp.h
#     mapper_types.h
#     message.c
#     message.h
#     mpr_signal.h
#     mpr_time.h
#     mpr_type.h
#     network.c
#     network.h
#     object.c
#     object.h
#     path.c
#     path.h
#     property.c
#     property.h
#     signal.c
#     slot.c
#     slot.h
#     table.c
#     table.h
#     thread_data.h
#     time.c
#     value.c
#     value.h
# )

# add_executable(crc32 crc32.c)
target_include_directories(app PRIVATE "src/libmapper-nxp/src")