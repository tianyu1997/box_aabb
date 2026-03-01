# ─── Experiments ──────────────────────────────────────────────────────────────

add_executable(exp1_coverage exp1_coverage.cpp)
target_link_libraries(exp1_coverage PRIVATE sbf)

add_executable(exp2_e2e_planning exp2_e2e_planning.cpp)
target_link_libraries(exp2_e2e_planning PRIVATE sbf)

add_executable(exp3_incremental exp3_incremental.cpp)
target_link_libraries(exp3_incremental PRIVATE sbf)

add_executable(diagnose_table diagnose_table.cpp)
target_link_libraries(diagnose_table PRIVATE sbf)

add_executable(diag_per_pair diag_per_pair.cpp)
target_link_libraries(diag_per_pair PRIVATE sbf)

add_executable(diag_anchor_aabb diag_anchor_aabb.cpp)
target_link_libraries(diag_anchor_aabb PRIVATE sbf)

add_executable(diag_aabb_timing diag_aabb_timing.cpp)
target_link_libraries(diag_aabb_timing PRIVATE sbf)
