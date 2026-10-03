# nvtx3-config.cmake - 手动提供 CUDA::nvtx3 目标
# 假设 CUDA 安装在 /usr/local/cuda，如需调整请修改路径

set(nvtx3_INCLUDE_DIRS /usr/local/cuda/include)
set(nvtx3_LIBRARIES "")

# 创建 IMPORTED 接口目标，名称必须是 CUDA::nvtx3
add_library(CUDA::nvtx3 INTERFACE IMPORTED)
set_target_properties(CUDA::nvtx3 PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${nvtx3_INCLUDE_DIRS}"
)

# 如果确实需要 libnvToolsExt.so，取消下面一行注释并注释掉上面 add_library 行
# add_library(CUDA::nvtx3 SHARED IMPORTED)
# set_target_properties(CUDA::nvtx3 PROPERTIES
#   IMPORTED_LOCATION /usr/local/cuda/lib64/libnvToolsExt.so
#   INTERFACE_INCLUDE_DIRECTORIES /usr/local/cuda/include
# )

# 标记包已被找到
set(nvtx3_FOUND TRUE)
