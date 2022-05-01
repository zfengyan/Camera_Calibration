# Android should use 3.1 or above
cmake_minimum_required(VERSION 3.1)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME "3rd_${MODULE_NAME}")
project(${PROJECT_NAME})


add_library(${PROJECT_NAME} STATIC
        logging.h
        logging.cc
        )


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party")

set(GLOG_INCLUDE_DIR  ${CMAKE_CURRENT_LIST_DIR}/..)
target_include_directories(${PROJECT_NAME} PUBLIC ${GLOG_INCLUDE_DIR})
