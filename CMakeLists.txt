cmake_minimum_required(VERSION 3.24)
project(runway_cpsat VERSION 0.3.0 LANGUAGES CXX)

option(RUNWAY_WITH_ORTOOLS "Enable CP-SAT using installed OR-Tools" ON)
option(RUNWAY_BUILD_TESTS "Build CTest checks" ON)

add_library(runway_core STATIC
    src/sample_problem.cpp
    src/validator.cpp
    src/joint/sample_joint_problem.cpp
    src/joint/joint_validator.cpp
)
target_include_directories(runway_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(runway_core PROPERTIES
    CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
)

if(RUNWAY_WITH_ORTOOLS)
    # No FetchContent / third-party downloads: reuse the user's installed 9.14.
    find_package(ortools 9.14 CONFIG REQUIRED)

    # The previous 10-flight solver is kept for regression comparison.
    add_library(runway_cp_sat STATIC
        src/cp_sat_scheduler.cpp
        src/joint/cp_sat_joint.cpp
    )
    target_link_libraries(runway_cp_sat
        PUBLIC runway_core
        PRIVATE ortools::ortools
    )
    set_target_properties(runway_cp_sat PROPERTIES
        CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
    )

    # Upgraded demo: 20-flight joint route/runway scheduling.
    add_executable(runway_demo app/joint_main.cpp)
    target_link_libraries(runway_demo PRIVATE runway_cp_sat)
    set_target_properties(runway_demo PROPERTIES
        CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
    )

    # Backwards-compatible 10-flight demonstration.
    add_executable(runway_legacy_demo app/main.cpp)
    target_link_libraries(runway_legacy_demo PRIVATE runway_cp_sat)
    set_target_properties(runway_legacy_demo PROPERTIES
        CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
    )
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(runway_core PRIVATE -Wall -Wextra -Wpedantic)
    if(RUNWAY_WITH_ORTOOLS)
        target_compile_options(runway_cp_sat PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

if(RUNWAY_BUILD_TESTS)
    enable_testing()
    add_executable(runway_core_test tests/core_test.cpp)
    target_link_libraries(runway_core_test PRIVATE runway_core)
    set_target_properties(runway_core_test PROPERTIES
        CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
    )
    add_test(NAME runway_core COMMAND runway_core_test)

    add_executable(runway_joint_core_test tests/joint_core_test.cpp)
    target_link_libraries(runway_joint_core_test PRIVATE runway_core)
    set_target_properties(runway_joint_core_test PROPERTIES
        CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
    )
    add_test(NAME runway_joint_core COMMAND runway_joint_core_test)

    if(RUNWAY_WITH_ORTOOLS)
        add_executable(runway_solver_test tests/solver_test.cpp)
        target_link_libraries(runway_solver_test PRIVATE runway_cp_sat)
        set_target_properties(runway_solver_test PROPERTIES
            CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
        )
        add_test(NAME runway_solver COMMAND runway_solver_test)
        set_tests_properties(runway_solver PROPERTIES TIMEOUT 35)

        add_executable(runway_joint_solver_test tests/joint_solver_test.cpp)
        target_link_libraries(runway_joint_solver_test PRIVATE runway_cp_sat)
        set_target_properties(runway_joint_solver_test PROPERTIES
            CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
        )
        add_test(NAME runway_joint_solver COMMAND runway_joint_solver_test)
        set_tests_properties(runway_joint_solver PROPERTIES TIMEOUT 40)

        add_executable(runway_joint_infeasible_test tests/joint_infeasible_test.cpp)
        target_link_libraries(runway_joint_infeasible_test PRIVATE runway_cp_sat)
        set_target_properties(runway_joint_infeasible_test PROPERTIES
            CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO
        )
        add_test(NAME runway_joint_infeasible COMMAND runway_joint_infeasible_test)
        set_tests_properties(runway_joint_infeasible PROPERTIES TIMEOUT 15)
    endif()
endif()
