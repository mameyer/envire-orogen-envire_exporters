find_package(LibInit QUIET)
if(LIB_INIT_FOUND)
    init_module(
        SOURCES
            MLSMapKalmanExporter.cpp
            MLSMapPrecalculatedExporter.cpp
        HEADERS
            MLSMapKalmanExporter.hpp
            MLSMapPrecalculatedExporter.hpp
        DEPS_PKGCONFIG
            lib_init
    )
endif()
