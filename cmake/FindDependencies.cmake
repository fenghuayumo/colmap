if(COLMAP_FIND_QUIETLY)
    set(COLMAP_FIND_TYPE QUIET)
else()
    set(COLMAP_FIND_TYPE REQUIRED)
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.30")
    cmake_policy(SET CMP0167 NEW)
endif()


find_package(Boost ${COLMAP_FIND_TYPE} COMPONENTS
             graph
             program_options
             system)

find_package(Eigen3 3.4 ${COLMAP_FIND_TYPE})
# find_package(SuiteSparse COMPONENTS CHOLMOD REQUIRED)

find_package(FreeImage ${COLMAP_FIND_TYPE})

find_package(FLANN ${COLMAP_FIND_TYPE})
find_package(LZ4 ${COLMAP_FIND_TYPE})

find_package(Metis ${COLMAP_FIND_TYPE})

find_package(Glog ${COLMAP_FIND_TYPE})
if(DEFINED glog_VERSION_MAJOR)
  # Older versions of glog don't export version variables.
  add_definitions("-DGLOG_VERSION_MAJOR=${glog_VERSION_MAJOR}")
  add_definitions("-DGLOG_VERSION_MINOR=${glog_VERSION_MINOR}")
endif()

find_package(SQLite3 ${COLMAP_FIND_TYPE})

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL ${COLMAP_FIND_TYPE})

find_package(Glew ${COLMAP_FIND_TYPE})

find_package(Git)

find_package(Ceres ${COLMAP_FIND_TYPE} COMPONENTS SuiteSparse)
if(NOT TARGET Ceres::ceres)
    # Older Ceres versions don't come with an imported interface target.
    add_library(Ceres::ceres INTERFACE IMPORTED)
    target_include_directories(
        Ceres::ceres INTERFACE ${CERES_INCLUDE_DIRS})
    target_link_libraries(
        Ceres::ceres INTERFACE ${CERES_LIBRARIES})
endif()
find_package(SuiteSparse COMPONENTS CHOLMOD REQUIRED)
if(TESTS_ENABLED)
    find_package(GTest ${COLMAP_FIND_TYPE})
endif()

if(OPENMP_ENABLED)
    find_package(OpenMP QUIET)
endif()

if(OPENMP_ENABLED AND OPENMP_FOUND)
    message(STATUS "Enabling OpenMP support")
    add_definitions("-DCOLMAP_OPENMP_ENABLED")
else()
    message(STATUS "Disabling OpenMP support")
endif()

if(CGAL_ENABLED)
    set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
    # We do not use CGAL data. This prevents an unnecessary warning by CMake.
    set(CGAL_DATA_DIR "unused")
    find_package(CGAL ${COLMAP_FIND_TYPE})
endif()

if(CGAL_FOUND)
    add_definitions("-DCOLMAP_CGAL_ENABLED")
    list(APPEND CGAL_LIBRARY ${CGAL_LIBRARIES})
    message(STATUS "Found CGAL")
    message(STATUS "  Includes : ${CGAL_INCLUDE_DIRS}")
    message(STATUS "  Libraries : ${CGAL_LIBRARY}")
    if(NOT TARGET CGAL)
        # Older CGAL versions don't come with an imported interface target.
        add_library(CGAL INTERFACE IMPORTED)
        target_include_directories(
            CGAL INTERFACE ${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIR})
        target_link_libraries(
            CGAL INTERFACE ${CGAL_LIBRARY} ${GMP_LIBRARIES})
    endif()
    list(APPEND COLMAP_LINK_DIRS ${CGAL_LIBRARIES_DIR})
endif()

if(NOT FETCH_POSELIB)
    find_package(PoseLib ${COLMAP_FIND_TYPE})
endif()

set(COLMAP_LINK_DIRS ${Boost_LIBRARY_DIRS})

set(CUDA_MIN_VERSION "7.0")
if(CUDA_ENABLED)
    find_package(CUDAToolkit)
    if (NOT CUDAToolkit_FOUND)
        message(WARNING "CUDA toolkit not found, building with CPU support only")
    else()
        if (COLMAP_MAX_CUDA_COMPATIBILITY)
            execute_process(COMMAND "${CUDAToolkit_NVCC_EXECUTABLE}" --list-gpu-arch 
                OUTPUT_VARIABLE LIST_GPU_ARCH 
                ERROR_QUIET)
        endif()

        if(NOT LIST_GPU_ARCH AND COLMAP_MAX_CUDA_COMPATIBILITY)
            message(WARNING "Cannot compile for max CUDA compatibility, nvcc does not support --list-gpu-arch")
            SET(COLMAP_MAX_CUDA_COMPATIBILITY OFF)
        endif()
        if(NOT COLMAP_MAX_CUDA_COMPATIBILITY)
            if(NOT CMAKE_CUDA_ARCHITECTURES)
                SET(CMAKE_CUDA_ARCHITECTURES 70;75;80)
            endif()
        else()
            # Build for maximum compatibility
            # https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
            set(CMAKE_CUDA_ARCHITECTURES "")

            # Extract list of arch and gencodes
            string(REPLACE "\r" "" LIST_GPU_ARCH ${LIST_GPU_ARCH})
            string(REPLACE "\n" ";" LIST_GPU_ARCH ${LIST_GPU_ARCH})

            execute_process(COMMAND "${CUDAToolkit_NVCC_EXECUTABLE}" --list-gpu-code 
                OUTPUT_VARIABLE LIST_GPU_CODE 
                ERROR_QUIET)
            string(REPLACE "\r" "" LIST_GPU_CODE ${LIST_GPU_CODE})
            string(REPLACE "\n" ";" LIST_GPU_CODE ${LIST_GPU_CODE})

            list(GET LIST_GPU_CODE 0 TARGET_GPU_CODE)
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=${TARGET_GPU_CODE}")

            set(IDX 0)
            foreach(GPU_ARCH ${LIST_GPU_ARCH})
                string(REGEX MATCH "compute_([0-9]+)" GPU_ARCH_VERSION "${GPU_ARCH}")
                list(APPEND CMAKE_CUDA_ARCHITECTURES "${CMAKE_MATCH_1}")
                list(GET LIST_GPU_CODE ${IDX} GPU_CODE)
                set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=${GPU_ARCH},code=${GPU_CODE}")
                math(EXPR IDX "${IDX}+1")
            endforeach()
            message("Set CUDA flags: " ${CMAKE_CUDA_FLAGS})
        endif()
        set(CUDA_FOUND ON)
        enable_language(CUDA)
        set(CMAKE_CUDA_STANDARD 17)
        set(CUDA_STANDARD 17)
    endif()
endif()

if(CUDA_ENABLED AND CUDA_FOUND)
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()

    add_definitions("-DCOLMAP_CUDA_ENABLED")

    # Do not show warnings if the architectures are deprecated.
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Wno-deprecated-gpu-targets")
    # Explicitly set PIC flags for CUDA targets.
    if(NOT IS_MSVC)
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --compiler-options -fPIC")
    endif()

    message(STATUS "Enabling CUDA support (version: ${CUDAToolkit_VERSION}, "
                    "archs: ${CMAKE_CUDA_ARCHITECTURES})")
else()
    set(CUDA_ENABLED OFF)
    message(STATUS "Disabling CUDA support")
endif()

if(GUI_ENABLED)
    find_package(Qt5 5.4 ${COLMAP_FIND_TYPE} COMPONENTS Core OpenGL Widgets)
    message(STATUS "Found Qt")
    message(STATUS "  Module : ${Qt5Core_DIR}")
    message(STATUS "  Module : ${Qt5OpenGL_DIR}")
    message(STATUS "  Module : ${Qt5Widgets_DIR}")
    if(Qt5_FOUND)
        # Qt5 was built with -reduce-relocations.
        if(Qt5_POSITION_INDEPENDENT_CODE)
            set(CMAKE_POSITION_INDEPENDENT_CODE ON)
            # Workaround for Qt5 CMake config bug under Ubuntu 20.04: https://gitlab.kitware.com/cmake/cmake/-/issues/16915
            if(TARGET Qt5::Core)
                get_property(core_options TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS)
                string(REPLACE "-fPIC" "" new_qt5_core_options "${core_options}")
                set_property(TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS ${new_qt5_core_options})
                set_property(TARGET Qt5::Core PROPERTY INTERFACE_POSITION_INDEPENDENT_CODE "ON")
                if(NOT IS_MSVC)
                    set(CMAKE_CXX_COMPILE_OPTIONS_PIE "-fPIC")
                endif()
            endif()
        endif()

        # Enable automatic compilation of Qt resource files.
        set(CMAKE_AUTORCC ON)
    endif()
endif()

if(GUI_ENABLED AND Qt5_FOUND)
    add_definitions("-DCOLMAP_GUI_ENABLED")
    message(STATUS "Enabling GUI support")
else()
    set(GUI_ENABLED OFF)
    message(STATUS "Disabling GUI support")
endif()

if(OPENGL_ENABLED)
    if(NOT GUI_ENABLED)
        message(STATUS "Disabling GUI also disables OpenGL")
        set(OPENGL_ENABLED OFF)
    else()
        add_definitions("-DCOLMAP_OPENGL_ENABLED")
        message(STATUS "Enabling OpenGL support")
    endif()
else()
    message(STATUS "Disabling OpenGL support")
endif()

set(GPU_ENABLED OFF)
if(OPENGL_ENABLED OR CUDA_ENABLED)
    add_definitions("-DCOLMAP_GPU_ENABLED")
    message(STATUS "Enabling GPU support (OpenGL: ${OPENGL_ENABLED}, CUDA: ${CUDA_ENABLED})")
    set(GPU_ENABLED ON)
endif()
