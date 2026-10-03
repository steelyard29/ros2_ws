if(NOT TARGET CUDA::nvtx3)
  add_library(CUDA::nvtx3 INTERFACE IMPORTED)
  set_target_properties(CUDA::nvtx3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "/usr/local/cuda/include"
  )
endif()

if(NOT TARGET magic_enum::magic_enum)
  add_library(magic_enum::magic_enum INTERFACE IMPORTED)
  set_target_properties(magic_enum::magic_enum PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "/home/cfly/magic_enum/include/magic_enum"
  )
endif()
