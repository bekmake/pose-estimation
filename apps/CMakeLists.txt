
#defining C++ version
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl -lutil")

#OpenCV
if (WIN32)
    set(OpenCV_DIR "" CACHE PATH "OpenCV installation folder")
endif ()

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

include(CTest)
enable_testing()

#Defining excutable file
add_executable(PoseEstimation PoseEstimation.cpp)

#Ataching OpenCV Libraries
target_link_libraries(PoseEstimation ${OpenCV_LIBS})

#Make it C++17
target_link_options(PoseEstimation PRIVATE "LINKER:-lutil")
target_compile_features(PoseEstimation PRIVATE cxx_std_17)

#add_subdirectory(libs) 

#Ataching headers
target_include_directories(PoseEstimation PRIVATE ./libs/include)
target_sources(PoseEstimation PRIVATE ./libs/src/utils.cpp)


#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#include(CPack)
