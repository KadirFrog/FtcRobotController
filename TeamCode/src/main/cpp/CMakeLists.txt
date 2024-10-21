# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

project("${CMAKE_PROJECT_NAME}")

add_library( # Specifies the name of the library.
        track_position

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        sources/track_position.cpp
        sources/Position.cpp
        )


#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY C:/Users/fnlg/StudioProjects/FtcRobotController/libs)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY C:/Users/fnlg/StudioProjects/FtcRobotController/libs)

# Specifies a path to native header files.
include_directories(header)