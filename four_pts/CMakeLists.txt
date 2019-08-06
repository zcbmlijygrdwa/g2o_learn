cmake_minimum_required (VERSION 2.6)
project(four_pts)
find_package( G2O REQUIRED )
find_package( Eigen3 REQUIRED )
find_package ( CSparse REQUIRED )

include_directories(
${EIGEN3_INCLUDE_DIR}
${CSPARSE_INCLUDE_DIR}
)

add_executable(four_pts four_pts.cpp)
target_link_libraries( four_pts
    g2o_core g2o_types_slam2d g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    ${CSPARSE_LIBRARY}
    )


