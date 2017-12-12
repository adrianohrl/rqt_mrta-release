cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(rqt_mrta)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  variant_topic_tools
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ${rqt_mrta_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rqt_gui
    rqt_gui_cpp
    variant_topic_tools
)

set (rqt_mrta_SRCS
  src/mrta/allocation.cpp
  src/mrta/architecture.cpp
  src/mrta/architecture_config.cpp
  src/mrta/history.cpp
  src/mrta/log.cpp
  src/mrta/monitor.cpp
  src/mrta/problem.cpp
  src/mrta/robot.cpp
  src/mrta/robot_monitor.cpp
  src/mrta/sample_holder.cpp
  src/mrta/state_monitor.cpp
  src/mrta/system.cpp
  src/mrta/task.cpp
  src/mrta/taxonomy.cpp
  src/rqt_mrta/architecture_combo_box.cpp
  src/rqt_mrta/config/application/application.cpp
  src/rqt_mrta/config/application/robot.cpp
  src/rqt_mrta/config/application/robots.cpp
  src/rqt_mrta/config/application/rqt_mrta_application.cpp
  src/rqt_mrta/config/application/rqt_mrta_package.cpp
  src/rqt_mrta/config/application/task.cpp
  src/rqt_mrta/config/application/tasks.cpp
  src/rqt_mrta/config/architecture/allocated_tasks.cpp
  src/rqt_mrta/config/architecture/allocations.cpp
  src/rqt_mrta/config/architecture/architecture.cpp
  src/rqt_mrta/config/architecture/busy_robots.cpp
  src/rqt_mrta/config/architecture/idle_robots.cpp
  src/rqt_mrta/config/architecture/incoming_tasks.cpp
  src/rqt_mrta/config/architecture/robots.cpp
  src/rqt_mrta/config/architecture/rqt_mrta_architecture.cpp
  src/rqt_mrta/config/architecture/tasks.cpp
  src/rqt_mrta/config/architecture/topic.cpp
  src/rqt_mrta/config/architecture/widget.cpp
  src/rqt_mrta/config/architecture/widgets.cpp
  src/rqt_mrta/config/arg.cpp
  src/rqt_mrta/config/args.cpp
  src/rqt_mrta/config/config.cpp
  src/rqt_mrta/config/configs.cpp
  src/rqt_mrta/config/include.cpp
  src/rqt_mrta/config/includes.cpp
  src/rqt_mrta/config/launch.cpp
  src/rqt_mrta/config/launches.cpp
  src/rqt_mrta/config/param.cpp
  src/rqt_mrta/config/param_interface.cpp
  src/rqt_mrta/config/params.cpp
  src/rqt_mrta/config/params_array.cpp
  src/rqt_mrta/define_application_widget.cpp
  src/rqt_mrta/define_application_wizard_page.cpp
  src/rqt_mrta/define_architecture_widget.cpp
  src/rqt_mrta/define_architecture_wizard_page.cpp
  src/rqt_mrta/define_parameters_widget.cpp
  src/rqt_mrta/define_parameters_wizard_page.cpp
  src/rqt_mrta/define_robots_widget.cpp
  src/rqt_mrta/define_robots_wizard_page.cpp
  src/rqt_mrta/labeled_status_widget.cpp
  src/rqt_mrta/new_application_wizard.cpp
  src/rqt_mrta/new_application_wizard_page.cpp
  src/rqt_mrta/new_architecture_wizard.cpp
  src/rqt_mrta/new_architecture_wizard_page.cpp
  src/rqt_mrta/param_item.cpp
  src/rqt_mrta/param_item_model.cpp
  src/rqt_mrta/param_tree_widget.cpp
  src/rqt_mrta/register_architecture_widget.cpp
  src/rqt_mrta/register_architecture_wizard_page.cpp
  src/rqt_mrta/robot_tree_widget.cpp
  src/rqt_mrta/rqt_mrta_plugin.cpp
  src/rqt_mrta/rqt_mrta_widget.cpp
  src/rqt_mrta/status_widget.cpp
  src/utilities/abstract_config.cpp
  src/utilities/exception.cpp
  src/utilities/message.cpp
  src/utilities/message_broker.cpp
  src/utilities/message_event.cpp
  src/utilities/message_field.cpp
  src/utilities/message_field_subscriber.cpp
  src/utilities/message_subscriber.cpp
  src/utilities/message_subscriber_registry.cpp
  src/utilities/ros_package.cpp
  src/utilities/simple_xml_settings.cpp
  src/utilities/xml_settings.cpp
)

set(rqt_mrta_HDRS
  include/mrta/allocation.h
  include/mrta/architecture.h
  include/mrta/architecture_config.h
  include/mrta/history.h
  include/mrta/log.h
  include/mrta/monitor.h
  include/mrta/problem.h
  include/mrta/robot.h
  include/mrta/robot_monitor.h
  include/mrta/sample_holder.h
  include/mrta/state_monitor.h
  include/mrta/system.h
  include/mrta/task.h
  include/mrta/taxonomy.h
  include/rqt_mrta/architecture_combo_box.h
  include/rqt_mrta/config/application/application.h
  include/rqt_mrta/config/application/robot.h
  include/rqt_mrta/config/application/robots.h
  include/rqt_mrta/config/application/rqt_mrta_application.h
  include/rqt_mrta/config/application/rqt_mrta_package.h
  include/rqt_mrta/config/application/task.h
  include/rqt_mrta/config/application/tasks.h
  include/rqt_mrta/config/architecture/allocated_tasks.h
  include/rqt_mrta/config/architecture/allocations.h
  include/rqt_mrta/config/architecture/architecture.h
  include/rqt_mrta/config/architecture/busy_robots.h
  include/rqt_mrta/config/architecture/idle_robots.h
  include/rqt_mrta/config/architecture/incoming_tasks.h
  include/rqt_mrta/config/architecture/robots.h
  include/rqt_mrta/config/architecture/rqt_mrta_architecture.h
  include/rqt_mrta/config/architecture/tasks.h
  include/rqt_mrta/config/architecture/topic.h
  include/rqt_mrta/config/architecture/widget.h
  include/rqt_mrta/config/architecture/widgets.h
  include/rqt_mrta/config/arg.h
  include/rqt_mrta/config/args.h
  include/rqt_mrta/config/configs.h
  include/rqt_mrta/config/config.h
  include/rqt_mrta/config/include.h
  include/rqt_mrta/config/includes.h
  include/rqt_mrta/config/launch.h
  include/rqt_mrta/config/launches.h
  include/rqt_mrta/config/param.h
  include/rqt_mrta/config/param_factory.h
  include/rqt_mrta/config/param_interface.h
  include/rqt_mrta/config/params.h
  include/rqt_mrta/config/params_array.h
  include/rqt_mrta/define_application_widget.h
  include/rqt_mrta/define_application_wizard_page.h
  include/rqt_mrta/define_architecture_widget.h
  include/rqt_mrta/define_architecture_wizard_page.h
  include/rqt_mrta/define_parameters_widget.h
  include/rqt_mrta/define_parameters_wizard_page.h
  include/rqt_mrta/define_robots_widget.h
  include/rqt_mrta/define_robots_wizard_page.h
  include/rqt_mrta/labeled_status_widget.h
  include/rqt_mrta/new_application_wizard.h
  include/rqt_mrta/new_application_wizard_page.h
  include/rqt_mrta/new_architecture_wizard.h
  include/rqt_mrta/new_architecture_wizard_page.h
  include/rqt_mrta/param_item.h
  include/rqt_mrta/param_item_model.h
  include/rqt_mrta/param_tree_widget.h
  include/rqt_mrta/register_architecture_widget.h
  include/rqt_mrta/register_architecture_wizard_page.h
  include/rqt_mrta/robot_tree_widget.h
  include/rqt_mrta/rqt_mrta_plugin.h
  include/rqt_mrta/rqt_mrta_widget.h
  include/rqt_mrta/status_widget.h
  include/utilities/abstract_config.h
  include/utilities/exception.h
  include/utilities/message.h
  include/utilities/message_broker.h
  include/utilities/message_event.h
  include/utilities/message_field.h
  include/utilities/message_field_subscriber.h
  include/utilities/message_subscriber.h
  include/utilities/message_subscriber_registry.h
  include/utilities/ros_package.h
  include/utilities/simple_xml_settings.h
  include/utilities/xml_settings.h
)

set(rqt_mrta_UIS
  src/Ui/define_application_widget.ui
  src/Ui/define_architecture_widget.ui
  src/Ui/define_parameters_widget.ui
  src/Ui/define_robots_widget.ui
  src/Ui/labeled_status_widget.ui
  src/Ui/register_architecture_widget.ui
  src/Ui/rqt_mrta_widget.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_mrta_MOCS ${rqt_mrta_HDRS})
  qt5_wrap_ui(rqt_mrta_UIS_H ${rqt_mrta_UIS})
else()
  qt4_wrap_cpp(rqt_mrta_MOCS ${rqt_mrta_HDRS})
  qt4_wrap_ui(rqt_mrta_UIS_H ${rqt_mrta_UIS})
endif()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${mrta_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${rqt_mrta_SRCS}
  ${rqt_mrta_MOCS}
  ${rqt_mrta_UIS_H}
)

add_dependencies(${PROJECT_NAME}
  ${mrta_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${mrta_LIBRARIES}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.png"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_mrta
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
