rock_library(fps_per_velocity
    SOURCES ImageAnalysisTool.cpp ImageFeartureDetectorTools.cpp
    HEADERS ImageAnalysisTool.hpp ImageFeartureDetectorTools.hpp
    DEPS_PKGCONFIG opencv)
    
rock_library(haze_removal
    SOURCES CLAHE.cpp 
    HEADERS CLAHE.hpp SingleImageHazeRemoval.hpp 
    DEPS_PKGCONFIG opencv)
    
rock_library(pocolog_tools
    SOURCES PocologTools.cpp
    HEADERS PocologTools.hpp
    DEPS_PKGCONFIG pocolog_cpp opencv)

rock_executable(extract_image_from_log extract_image_from_log.cpp
    DEPS pocolog_tools
    DEPS_PKGCONFIG opencv frame_helper pocolog_cpp
  )
target_link_libraries(extract_image_from_log gomp)
