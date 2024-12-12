
set(FOLDER_NAME "colmap_dvs")

if(IS_MSVC)
    add_compile_options("/bigobj")
endif()

set(OPTIONAL_LIBS)
if(CUDA_ENABLED)
    list(APPEND OPTIONAL_LIBS
        colmap_util_cuda
        colmap_mvs_cuda
    )
endif()
# if(GUI_ENABLED)
#     list(APPEND OPTIONAL_LIBS
#         colmap_ui
#     )
# endif()
if(MSVC)
   add_definitions("-DCOLMAP_BUILD_MAIN_LIB", "-DCOLMAP_BUILD_SHARED_LIBS")
endif()
# find_package(SuiteSparse REQUIRED)
include_directories(${SuiteSparse_CHOLMOD_INCLUDE_DIR})
COLMAP_ADD_DLL(
    NAME colmap_dvs
    SRCS
    colmap_lib.h colmap_lib.cpp
    sparse_reconstruct_controller.h sparse_reconstruct_controller.cpp
    PUBLIC_LINK_LIBS
        colmap_controllers
        colmap_estimators
        colmap_geometry
        colmap_optim
        colmap_scene
        colmap_util
        glomap
        PoseLib::PoseLib
        SuiteSparse::CHOLMOD
        ${OPTIONAL_LIBS}
    PRIVATE_LINK_LIBS
        Boost::boost
)
# COLMAP_ADD_EXECUTABLE(
#     NAME colmap_main
#     SRCS
#         feature.cc
#         sfm.cc
#         colmap.cc
#         database.cc
#         feature.cc
#         gui.cc
#         image.cc
#         model.cc
#         mvs.cc
#         sfm.cc
#         vocab_tree.cc
#     LINK_LIBS
#         colmap_controllers
#         colmap_retrieval
#         colmap_scene
#         colmap_sfm
#         colmap_util
#         ${OPTIONAL_LIBS}
#         Boost::boost
# )
set_target_properties(colmap_dvs PROPERTIES OUTPUT_NAME colmap_dvs)