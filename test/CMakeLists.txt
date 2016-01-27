rock_library(lib_testTools
    SOURCES TestTools.cpp
    HEADERS TestTools.hpp
    DEPS_PKGCONFIG pocolog_cpp opencv)
    
#rock_testsuite(testtools_suite TestTools_test.cpp 
#    DEPS fps_per_velocity lib_testTools
#    DEPS_PKGCONFIG pocolog_cpp frame_helper opencv)

rock_testsuite(test_image_analisys_suite ImageAnalysisTool_test.cpp
    DEPS fps_per_velocity haze_removal lib_testTools
    DEPS_PKGCONFIG pocolog_cpp frame_helper opencv)
 
    
