#include "../header/track_position.h"

using namespace std;

namespace track_position {
    #define ENCODER_TICKS_PER_ROTATION 8192
    #define WHEEL_DIAMETER_MM 60

    JavaVM *jvm;

    jmethodID LynxModule_getBulkData;
    jmethodID LynxModule_isNotResponding;
    jmethodID ErrorDetection_addErrorToErrorLog;
    jmethodID IMU_getZRotation;
    jmethodID DcMotorEx_getCurrentPosition;

    jobject deadWheelForwards;
    jobject deadWheelSidewards;
    jobject imu;
    jobject controlHub;
    jobject expansionHub;
    jobject errorDetection;

    bool should_stop = false;

    int encoder_forward_pos = 0;
    int encoder_sidewards_pos = 0;
    int encoder_forward_last_pos = 0;
    int encoder_sidewards_last_pos = 0;

    double y_position = 0;
    double x_position = 0;
    double rotation_rad = 0;
    double rotation_deg = 0;
    double rotation_deg_continuous = 0;
    double extra_rad = 0;
    int extra_degrees = 0;

    chrono::high_resolution_clock::time_point last_time_millis = chrono::high_resolution_clock::now();
    double forwards_speed = 0;
    double sidewards_speed = 0;
    double y_speed = 0;
    double x_speed = 0;
    double rotation_speed = 0;

    int loop_time = 0;

    extern "C" JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_init
            (JNIEnv *env, jobject thiz, jobject hardwareMap) {
        env->GetJavaVM(&jvm);

        _jclass *const Robot = env->FindClass("org/firstinspires/ftc/teamcode/Robot");
        _jclass *const DcMotorEx = env->FindClass("com/qualcomm/robotcore/hardware/DcMotorEx");

        _jmethodID *const HardwareMap_get = env->GetMethodID(env->GetObjectClass(hardwareMap), "get", "(Ljava/lang/Class;Ljava/lang/String;)Ljava/lang/Object;");

        errorDetection = env->NewGlobalRef(env->GetStaticObjectField(Robot,
                                                                     env->GetStaticFieldID(Robot,
                                                                                           "errorDetection",
                                                                                           "Lorg/firstinspires/ftc/teamcode/control/ErrorDetection;")));
        deadWheelForwards = env->NewGlobalRef(env->CallObjectMethod(hardwareMap, HardwareMap_get, DcMotorEx, env->NewStringUTF(string("rear_left").c_str())));
        deadWheelSidewards = env->NewGlobalRef(env->CallObjectMethod(hardwareMap, HardwareMap_get, DcMotorEx, env->NewStringUTF(string("front_right").c_str())));
        imu = env->NewGlobalRef(env->GetStaticObjectField(Robot, env->GetStaticFieldID(Robot, "imu1", "Lorg/firstinspires/ftc/teamcode/hardware/sensor/IMU;")));
        controlHub = env->NewGlobalRef(env->GetStaticObjectField(Robot, env->GetStaticFieldID(Robot, "controlHub", "Lcom/qualcomm/hardware/lynx/LynxModule;")));
        expansionHub = env->NewGlobalRef(env->GetStaticObjectField(Robot, env->GetStaticFieldID(Robot, "expansionHub", "Lcom/qualcomm/hardware/lynx/LynxModule;")));

        _jclass *const IMU = env->GetObjectClass(imu);
        _jclass *const LynxModule = env->GetObjectClass(controlHub);
        _jclass *const ErrorDetection = env->GetObjectClass(errorDetection);

        LynxModule_getBulkData = env->GetMethodID(LynxModule, "getBulkData",
                                                                    "()Lcom/qualcomm/hardware/lynx/LynxModule$BulkData;");
        LynxModule_isNotResponding = env->GetMethodID(LynxModule,
                                                                        "isNotResponding",
                                                                        "()Z");
        ErrorDetection_addErrorToErrorLog = env->GetMethodID(ErrorDetection,
                                                                               "addErrorToErrorLog",
                                                                               "(Ljava/lang/String;)V");
        IMU_getZRotation = env->GetMethodID(IMU, "getZRotation", "()D");
        DcMotorEx_getCurrentPosition = env->GetMethodID(DcMotorEx,
                                                                          "getCurrentPosition",
                                                                          "()I");

        should_stop = false;

        encoder_forward_pos = 0;
        encoder_sidewards_pos = 0;
        encoder_forward_last_pos = 0;
        encoder_sidewards_last_pos = 0;

        y_position = 0;
        x_position = 0;
        rotation_rad = 0;
        rotation_deg = 0;
        rotation_deg_continuous = 0;
        extra_rad = 0;
        extra_degrees = 0;

        last_time_millis = std::chrono::high_resolution_clock::now();
        forwards_speed = 0;
        sidewards_speed = 0;
        y_speed = 0;
        x_speed = 0;
        rotation_speed = 0;

        /*
        _jmethodID *const DcMotorEx_setDirection = env->GetMethodID(DcMotorEx, "setDirection", "(Lcom/qualcomm/robotcore/hardware/DcMotorSimple$Direction;)V");
        _jclass *const Direction = env->FindClass("com/qualcomm/robotcore/hardware/DcMotorSimple$Direction");
        _jobject *const Direction_REVERSE = env->GetStaticObjectField(Direction, env->GetStaticFieldID(Direction, "REVERSE", "Lcom/qualcomm/robotcore/hardware/DcMotorSimple$Direction;"));
        env->CallVoidMethod(deadWheelSidewards, DcMotorEx_setDirection, Direction_REVERSE);
         */

        _jmethodID *const DcMotorEx_setMode = env->GetMethodID(DcMotorEx, "setMode", "(Lcom/qualcomm/robotcore/hardware/DcMotor$RunMode;)V");
        _jclass *const RunMode = env->FindClass("com/qualcomm/robotcore/hardware/DcMotor$RunMode");
        _jobject *const RunMode_STOP_AND_RESET_ENCODER = env->GetStaticObjectField(RunMode, env->GetStaticFieldID(RunMode, "STOP_AND_RESET_ENCODER", "Lcom/qualcomm/robotcore/hardware/DcMotor$RunMode;"));
        _jobject *const RunMode_RUN_WITHOUT_ENCODER = env->GetStaticObjectField(RunMode, env->GetStaticFieldID(RunMode, "RUN_WITHOUT_ENCODER", "Lcom/qualcomm/robotcore/hardware/DcMotor$RunMode;"));
        env->CallVoidMethod(deadWheelForwards, DcMotorEx_setMode, RunMode_STOP_AND_RESET_ENCODER);
        env->CallVoidMethod(deadWheelSidewards, DcMotorEx_setMode, RunMode_STOP_AND_RESET_ENCODER);
        env->CallVoidMethod(deadWheelForwards, DcMotorEx_setMode, RunMode_RUN_WITHOUT_ENCODER);
        env->CallVoidMethod(deadWheelSidewards, DcMotorEx_setMode, RunMode_RUN_WITHOUT_ENCODER);

        thread([] {
            JNIEnv *env;
            jvm->AttachCurrentThread(reinterpret_cast<JNIEnv **> (&env), nullptr);


            while (!should_stop) {
                env->DeleteLocalRef(env->CallObjectMethod(controlHub, LynxModule_getBulkData));
                if (env->CallBooleanMethod(controlHub, LynxModule_isNotResponding) == JNI_TRUE) {
                    env->CallVoidMethod(errorDetection, ErrorDetection_addErrorToErrorLog,
                                        env->NewStringUTF(
                                                string("control hub did not respond").c_str()));
                }

                env->DeleteLocalRef(env->CallObjectMethod(expansionHub, LynxModule_getBulkData));
                if (env->CallBooleanMethod(expansionHub, LynxModule_isNotResponding) == JNI_TRUE) {
                    env->CallVoidMethod(errorDetection, ErrorDetection_addErrorToErrorLog,
                                        env->NewStringUTF(
                                                string("expansion hub did not respond").c_str()));
                }

                double previousRobotRotationDeg = rotation_deg;
                double previousRobotPositionY = y_position;
                double previousRobotPositionX = x_position;

                double rotationRadNotFinal =
                        -static_cast<double>(env->CallDoubleMethod(imu, IMU_getZRotation)) +
                        extra_rad;
                if (rotationRadNotFinal > M_PI) {
                    rotation_rad = rotationRadNotFinal - M_PI * 2;
                } else if (rotationRadNotFinal < -M_PI) {
                    rotation_rad = rotationRadNotFinal + M_PI * 2;
                } else {
                    rotation_rad = rotationRadNotFinal;
                }

                encoder_forward_pos = static_cast<int>(env->CallIntMethod(deadWheelForwards,
                                                                          DcMotorEx_getCurrentPosition));
                encoder_sidewards_pos = static_cast<int>(env->CallIntMethod(deadWheelSidewards,
                                                                            DcMotorEx_getCurrentPosition));

                rotation_deg = rotation_rad * 180 / M_PI;

                // convert degrees to continuous degrees system
                if (previousRobotRotationDeg > 90 && rotation_deg < -90) {
                    extra_degrees += 360;
                } else if (previousRobotRotationDeg < -90 && rotation_deg > 90) {
                    extra_degrees -= 360;
                }
                rotation_deg_continuous = rotation_deg + extra_degrees;

                double deltaT = (double) (chrono::duration_cast<chrono::milliseconds>(
                        chrono::high_resolution_clock::now() - last_time_millis).count()) / 1000.0;

                double sinus = sin(rotation_rad);
                double cosinus = cos(rotation_rad);

                double deltaForward = encoder_forward_last_pos - encoder_forward_pos;
                double deltaSidewards = encoder_sidewards_last_pos - encoder_sidewards_pos;

                double deltaY = ((-deltaForward * cosinus + deltaSidewards * sinus) /
                                 ENCODER_TICKS_PER_ROTATION) * WHEEL_DIAMETER_MM * M_PI;
                double deltaX = ((-deltaForward * sinus - deltaSidewards * cosinus) /
                                 ENCODER_TICKS_PER_ROTATION) * WHEEL_DIAMETER_MM * M_PI;

                y_position += deltaY;
                x_position += deltaX;

                forwards_speed =
                        ((deltaForward / deltaT) / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM *
                         M_PI + forwards_speed) / 2;
                sidewards_speed = ((deltaSidewards / deltaT) / ENCODER_TICKS_PER_ROTATION *
                                   WHEEL_DIAMETER_MM * M_PI + sidewards_speed) / 2;
                y_speed = (((y_position - previousRobotPositionY) / deltaT) /
                           ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * M_PI + y_speed) / 2;
                x_speed = (((x_position - previousRobotPositionX) / deltaT) /
                           ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * M_PI + x_speed) / 2;
                rotation_speed =
                        (((rotation_deg - previousRobotRotationDeg) / deltaT) + rotation_speed) / 2;

                encoder_forward_last_pos = encoder_forward_pos;
                encoder_sidewards_last_pos = encoder_sidewards_pos;

                last_time_millis = chrono::high_resolution_clock::now();
            }

            env->DeleteGlobalRef(deadWheelForwards);
            env->DeleteGlobalRef(deadWheelSidewards);
            env->DeleteGlobalRef(imu);
            env->DeleteGlobalRef(controlHub);
            env->DeleteGlobalRef(expansionHub);

            jvm->DetachCurrentThread();
        }).detach();
    }

    extern "C"
    JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_stopTracking
            (JNIEnv *env, jobject thiz) {
        should_stop = true;
    }


    extern "C" JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_writeStoredDataToVariables
            (JNIEnv *env, jobject thiz, jdouble xPosition, jdouble yPosition, jdouble rotationRad,
             jint extraDegrees) {
        x_position = xPosition;
        y_position = yPosition;
        extra_rad = rotationRad;
        extra_degrees = extraDegrees;
    }

    extern "C" JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_setPosition
            (JNIEnv *env, jobject thiz, jdouble xPosition, jdouble yPosition, jdouble rotation) {
        x_position = xPosition;
        y_position = yPosition;
        extra_degrees = (int) floor(abs(rotation) / 360) * 360 * ((rotation > 0) - (rotation < 0));

        rotation = fmod(rotation, 360.0);
        if (rotation > 180.0) {
            rotation -= 360.0;
        } else if (rotation < -180.0) {
            rotation += 360.0;
        }
        extra_rad = rotation * (M_PI / 180);
    }

    extern "C"
    JNIEXPORT void JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_setCoordinates
            (JNIEnv *env, jobject thiz, jdouble xPosition, jdouble yPosition) {
        x_position = xPosition;
        y_position = yPosition;
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getForwardMm
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> ((double) encoder_forward_pos / ENCODER_TICKS_PER_ROTATION *
                                     WHEEL_DIAMETER_MM * M_PI);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getSidewardsMm
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> ((double) encoder_sidewards_pos / ENCODER_TICKS_PER_ROTATION *
                                     WHEEL_DIAMETER_MM * M_PI);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getXPosition
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (x_position);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getYPosition
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (y_position);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getRotation
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (rotation_deg_continuous);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getRotationRad
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (rotation_rad);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getForwardSpeed
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (forwards_speed);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getSidewardsSpeed
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (sidewards_speed);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getXSpeed
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (x_speed);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getYSpeed
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (y_speed);
    }

    extern "C"
    JNIEXPORT jdouble JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getRotationSpeed
            (JNIEnv *env, jobject thiz) {
        return static_cast<jdouble> (rotation_speed);
    }

    extern "C"
    JNIEXPORT jint JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getExtraDegrees
            (JNIEnv *env, jobject thiz) {
        return static_cast<jint> (extra_degrees);
    }

    extern "C"
    JNIEXPORT jint JNICALL
    Java_org_firstinspires_ftc_teamcode_hardware_sensor_TrackPosition_getLoopTime
            (JNIEnv *env, jobject thiz) {
        return static_cast<jint> (loop_time);
    }

    Position get_position() {
        return {x_position, y_position, rotation_deg_continuous};
    }

    double get_rotation_rad() {
        return rotation_rad;
    }
}