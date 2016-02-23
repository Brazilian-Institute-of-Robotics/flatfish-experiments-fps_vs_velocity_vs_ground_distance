rock_testsuite(pocolog_tools_suite PocologTools_test.cpp
    HEADERS TestTools.hpp 
    DEPS fps_per_velocity pocolog_tools
    DEPS_PKGCONFIG frame_helper opencv)

rock_testsuite(test_image_fearture_detector_suite ImageFeartureDetector_test.cpp
    HEADERS TestTools.hpp 
    DEPS fps_per_velocity
    DEPS_PKGCONFIG opencv)

rock_testsuite(test_image_analisys_suite ImageAnalysis_test.cpp
    HEADERS TestTools.hpp
    DEPS fps_per_velocity haze_removal pocolog_tools
    DEPS_PKGCONFIG frame_helper)
