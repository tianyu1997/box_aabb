if(SBF_WITH_PYTHON)
    find_package(pybind11 REQUIRED)

    pybind11_add_module(pysbf bindings.cpp)
    target_link_libraries(pysbf PRIVATE sbf)
    target_include_directories(pysbf PRIVATE ${CMAKE_SOURCE_DIR}/include)

    # Install to Python site-packages
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import site; print(site.getusersitepackages())"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    install(TARGETS pysbf
        LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/lib
    )
endif()
