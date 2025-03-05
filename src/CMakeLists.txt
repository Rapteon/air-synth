#set(HEADER_LIST "${CMAKE_SOURCE_DIR}/include/simple-lib/simple.h")
#
#add_library(simple-lib simple.cpp ${HEADER_LIST})
#
#target_include_directories(simple-lib PUBLIC ../include)
#
#target_compile_features(simple-lib PUBLIC cxx_std_17)
#
#source_group(
#    TREE "${PROJECT_SOURCE_DIR}/include"
#    PREFIX "Header Files"
#    FILES ${HEADER_LIST}
#)
