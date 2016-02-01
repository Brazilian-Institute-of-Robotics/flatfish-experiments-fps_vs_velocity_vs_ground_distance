rock_library(fps_per_velocity
    SOURCES ImageAnalysisTool.cpp ImageFeartureDetectorTools.cpp
    HEADERS ImageAnalysisTool.hpp ImageFeartureDetectorTools.hpp
    DEPS_PKGCONFIG opencv)
    
rock_library(haze_removal
    SOURCES CLAHE.cpp HistogramEqualization.cpp
    HEADERS CLAHE.hpp HistogramEqualization.hpp SingleImageHazeRemoval.hpp 
    DEPS_PKGCONFIG opencv)

#rock_executable(fps_per_velocity_bin
#    SOURCES Main.cpp
#    LIBS fps_per_velocity)

