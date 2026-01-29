cmake_minimum_required(VERSION 3.16)

project(ServoDemo VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ServoDemo
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ecatwidget.h ecatwidget.cpp ecatwidget.ui
        ecatmanager.h ecatmanager.cpp
        ecatmaster.h ecatmaster.cpp
        slave.h
        servol7nh.h servol7nh.cpp
        cia402.h
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET ServoDemo APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(ServoDemo SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(ServoDemo
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(ServoDemo PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
)

# soem
target_include_directories(${PROJECT_NAME} PRIVATE
    ../../gneetech_lib/SOEM/soem
    ../../gneetech_lib/SOEM/osal
    ../../gneetech_lib/SOEM/osal/win32
    ../../gneetech_lib/SOEM/oshw/win32
    ../../gneetech_lib/SOEM/oshw/win32/wpcap/include
    ../../gneetech_lib/SOEM/soem
)

target_link_directories(${PROJECT_NAME} PRIVATE
    ../../gneetech_lib/SOEM/build
    ../../gneetech_lib/SOEM/oshw/win32/wpcap/Lib/x64
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    soem
    Packet
    wpcap
    winmm
    ws2_32
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    __STDC_LIMIT_MACROS     #soem
    __STDC_CONSTANT_MACROS  #soem
    WIN32_LEAN_AND_MEAN     #soem
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.ServoDemo)
endif()
set_target_properties(ServoDemo PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS ServoDemo
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ServoDemo)
endif()
