#ifndef FTCROBOTCONTROLLER_TRACK_POSITION_H
#define FTCROBOTCONTROLLER_TRACK_POSITION_H

#include <jni.h>
#include <cmath>
#include <chrono>
#include <thread>

#include "Position.h"

namespace track_position {
    // Tracks the position of the robot in x, y and a continuous rotation
    // system. It also calculates some other data about the robots
    // movement. All these data is stored in the variables
    extern "C" JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_init
            (JNIEnv *env, jobject thiz, jobject hardwareMap);

    // Stops trackPosition()
    extern "C"
    JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_stopTracking
            (JNIEnv *env, jobject thiz);

    // Writes the data given to the variables. Used in combination
    // with the LastPositionStorage to transfer data from one OpMode
    // to another
    extern "C" JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_writeStoredDataToVariables
            (JNIEnv *env, jobject thiz, jdouble xPosition, jdouble yPosition, jdouble rotationRad,
             jint extraDegreesPara);

    // Used to set the robots position to a specific position
    extern "C" JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_setPosition
            (JNIEnv *env, jobject thiz, jdouble xPosition, jdouble yPosition, jdouble rotation);

    extern "C"
    JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_setCoordinates
        (JNIEnv *env, jobject thiz, jdouble xPosition, jdouble yPosition);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getForwardMm
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getSidewardsMm
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getXPosition
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getYPosition
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getRotation
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getRotationRad
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getForwardSpeed
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getSidewardsSpeed
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getXSpeed
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getYSpeed
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getRotationSpeed
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jint JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getExtraDegrees
            (JNIEnv *env, jobject thiz);

    extern "C"
    JNIEXPORT jint JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getLoopTime
            (JNIEnv *env, jobject thiz);

    Position get_position();

    double get_rotation_rad();
}

#endif //FTCROBOTCONTROLLER_TRACK_POSITION_H