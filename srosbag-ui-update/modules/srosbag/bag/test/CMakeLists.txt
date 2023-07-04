
SET(CMAKE_BUILD_TYPE “Debug”)


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
        cv_bridge
        image_transport
        message_generation
        message_runtime
#        tsdf_slam
        )
find_package(PCL REQUIRED)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        include ${PCL_INCLUDE_DIRS}
        /home/dzl/tsdf_salm_ws/devel/include/tsdf_slam
)
link_directories(${PCL_LIBRARY_DIRS})


if (QT_BUILD)

    set(QT5_ROOT_PATH CACHE PATH "/home/anxixu/Qt5.12.1/5.12.1")
    set(CMAKE_PREFIX_PATH ${QT5_ROOT_PATH})
    if (QT5_ROOT_PATH)
        list(APPEND CMAKE_PREFIX_PATH ${QT5_ROOT_PATH})
    endif ()

    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    SET(CMAKE_AUTOUIC ON)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include/QtWidgets)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include/QtGui)
    include_directories(/home/anxixu/Qt5.12.1/5.12.1/gcc_64/include/QtCore)
    #find_package(QT5_ROOT_PATH )
    find_package(Qt5 COMPONENTS Core Gui Qml Quick Widgets REQUIRED)
    include_directories(/home/anxixu/anxixu/sros_ui_update/modules/srosbag/bag/include/qt)
    add_definitions(-DQT_NO_VERSION_TAGGING)
endif ()


#add_message_files(FILES
#        CustomPoint.msg
#        CustomMsg.msg
#        #   Message2.msg
#        )
#
#generate_messages(DEPENDENCIES
#        std_msgs
#        )

catkin_package(CATKIN_DEPENDS
        roscpp rospy std_msgs message_runtime
        pcl_ros
        )

#add_dependencies(test livox_CustomMsg_gencpp)

add_executable(test
        test.cpp
        ../../../../modules/srosbag/bag/src/qt/qtshow.cpp
        ../../../../modules/srosbag/bag/include/qt/dialog.h
        ../../../../modules/srosbag/bag/include/qt/playbag.h
        ../../../../modules/srosbag/bag/include/qt/progressbar.h

        ../../../../modules/srosbag/bag/include/qt/qtshow.h

        )
add_dependencies(test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(laser_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(test
        PUBLIC
        ${BAG_INCLUDE_DIR}
        )


target_link_libraries(test
        PUBLIC
        msg_bag
        my_qt
        Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Widgets
        ${catkin_LIBRARIES}
        )

# add_executable(test_camera
#         test_camera.cpp)
# target_include_directories(test_camera
#         PUBLIC
#         ${BAG_INCLUDE_DIR})
# target_link_libraries(test_camera
#         PUBLIC
#         msg_bag)
