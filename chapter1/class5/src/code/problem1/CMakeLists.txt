project(HELLO)
set(SRC_LIST main.c annai.c)
include_directories(./headfile)
add_executable(hello ${SRC_LIST})
