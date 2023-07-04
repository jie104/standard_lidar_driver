if(QT_BUILD)
    add_library(my_qt STATIC "")


    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON) 


    find_package(Qt5 COMPONENTS Core Gui Qml Quick Widgets REQUIRED)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include/QtWidgets)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include/QtGui)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include/QtCore)
    include_directories(include/qt)


    target_sources(my_qt
                   PUBLIC
   
                   ${CMAKE_CURRENT_LIST_DIR}/playbag.cpp
                   ${CMAKE_CURRENT_LIST_DIR}/progressbar.cpp
                   ${CMAKE_CURRENT_LIST_DIR}/dialog.cpp
                   ${CMAKE_CURRENT_LIST_DIR}/qtshow.cpp
                   )
    
    target_compile_options(my_qt PRIVATE -fPIC)

    target_include_directories(my_qt
                              PUBLIC
                   $<BUILD_INTERFACE:${BAG_INCLUDE_DIR}>
                              Qt5::Gui
                              )
    target_link_libraries(my_qt
                Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Widgets)






endif()
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
message_filters
rospy
std_msgs
pcl_ros
pcl_conversions
laser_geometry
tf
)
find_package(PCL REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})