# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2021-2025, The OpenROAD Authors

include("openroad")

# For Multithread
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_FFT2D_PTHREADS=1 -DFFT_2D_MAX_THREADS=16 -O3 ${OpenMP_CXX_FLAGS}")

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3 REQUIRED)
find_package(ortools REQUIRED)
find_package(LEMON NAMES LEMON lemon REQUIRED)
find_package(OpenMP REQUIRED)

message(STATUS "Found OR-Tools: ${ortools_DIR} (version: ${ortools_VERSION})")

swig_lib(NAME      gpl
         NAMESPACE gpl
         I_FILE    src/replace.i
         SWIG_INCLUDES ${ODB_HOME}/src/swig/common
                       ${ODB_HOME}/src/swig/tcl
                       ${ODB_HOME}/include
         SCRIPTS   src/replace.tcl
)

add_library(gpl_lib
  src/AbstractGraphics.cpp
  src/replace.cpp
  src/initialPlace.cpp
  src/nesterovPlace.cpp
  src/placerBase.cpp
  src/nesterovBase.cpp
  src/fft.cpp
  src/fftsg.cpp
  src/fftsg2d.cpp
  src/routeBase.cpp
  src/timingBase.cpp
  src/graphicsNone.cpp
  src/solver.cpp
  src/mbff.cpp
)

target_sources(gpl
  PRIVATE
    src/MakeReplace.cpp
    src/graphicsImpl.cpp
)

messages(TARGET gpl)
messages(TARGET gpl_lib)

target_include_directories(gpl
  PUBLIC
    include
)

target_include_directories(gpl_lib
  PUBLIC
    include
    ${LEMON_INCLUDE_DIRS}
)

target_link_libraries(gpl_lib
    utl_lib
    Eigen3::Eigen
    odb
    OpenSTA
    rsz_lib
    grt_lib
    ortools::ortools
    Threads::Threads
    OpenMP::OpenMP_CXX
)

target_link_libraries(gpl
  PRIVATE
    gpl_lib
    utl_lib
    Eigen3::Eigen
    gui
    odb
    OpenSTA
    rsz
    grt
    ortools::ortools
    Threads::Threads
    OpenMP::OpenMP_CXX
)

# Allow users to use GPU or not
option(GPU "Enable GPU" OFF)
if (GPU)
  message(STATUS "GPU is enabled")
  # find_package(CUDA) should be replaced by "FindCUDAToolkit" for CMAKE verison >= 3.17. 
  # See https://cmake.org/cmake/help/latest/module/FindCUDAToolkit.html. 
  # find_package(CUDA) or FindCUDA is deprecated since vision 3.10.
  find_package(CUDA QUIET COMPONENTS Core Widgets)
  if (CUDA_FOUND)
    message(STATUS "CUDA is found")
    enable_language(CUDA)
    target_sources(gpl PRIVATE src/gpuSolver.cu)
    target_link_libraries(gpl
      PRIVATE
        ${CUDA_LIBRARIES}
        ${CUDA_CUBLAS_LIBRARIES}
        ${CUDA_cusparse_LIBRARY}
        ${CUDA_cusolver_LIBRARY}
    )
    target_compile_definitions( gpl
    PRIVATE
      ENABLE_GPU
    )
  else()
    message(STATUS "CUDA is not found")
  endif()
else()
  message(STATUS "GPU is not enabled")
endif()

if (Python3_FOUND AND BUILD_PYTHON)
  swig_lib(NAME          gpl_py
           NAMESPACE     gpl
           LANGUAGE      python
           I_FILE        src/replace-py.i
           SWIG_INCLUDES ${PROJECT_SOURCE_DIR}/include/gpl
           SCRIPTS       ${CMAKE_CURRENT_BINARY_DIR}/gpl_py.py
  )

  target_include_directories(gpl_py
    PUBLIC
      include
      
  )

  target_link_libraries(gpl_py
    PUBLIC
      utl_lib
      Eigen3::Eigen
      gui
      odb
      OpenSTA
      rsz
      grt
  )

endif()

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()
