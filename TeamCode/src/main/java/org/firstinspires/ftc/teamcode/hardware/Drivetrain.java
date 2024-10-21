package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.PIDController;


@SuppressWarnings({"unused", "UnusedReturnValue"})
@Config
public class Drivetrain {
    double motorFrontLeftTargetSpeed = 0;
    double motorFrontRightTargetSpeed = 0;
    double motorRearLeftTargetSpeed = 0;
    double motorRearRightTargetSpeed = 0;

    double lastMotorFrontLeftTargetSpeed = 0;
    double lastMotorFrontRightTargetSpeed = 0;
    double lastMotorRearLeftTargetSpeed = 0;
    double lastMotorRearRightTargetSpeed = 0;

    long lastTimeLoop = System.currentTimeMillis();

    PIDController pidForward = new PIDController(1, 0.3, 10);
    PIDController pidSidewards = new PIDController(1, 0.3, 10);
    public static double P = 0.001;
    public static double I = 0.0001;
    public static double D = 0.0001;
    PIDController pidTurn = new PIDController(P, I, D);
    PIDController pidDistanceForward = new PIDController(0.0001, 0, 0.000025);
    PIDController pidDistanceTurn = new PIDController(0.0004, 0, 0.0001);
    PIDController pidDriveToDistance = new PIDController(0.0001, 0, 0.000025);

    double maxTargetSpeed = 0;
    double minPowerNeededToTurn = 0.2;

    double forwards = 0;
    double sidewards = 0;
    public double turn = 0;
    double rotationOffset = 0;
    double timeFromZeroToMaxSpeed = 1;

    boolean motorFrontLeftOnFire = false;
    boolean motorFrontRightOnFire = false;
    boolean motorRearLeftOnFire = false;
    boolean motorRearRightOnFire = false;

    boolean useHeadless = true;
    boolean useDriveMotorEncoderSupport = true;
    boolean useGyroSupport = true;
    boolean useDeadWheelSupport = true;
    boolean useControlledAcceleration = true;

    public DcMotorEx motorFrontLeft; // motor
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorRearLeft;
    public DcMotorEx motorRearRight;


    public Drivetrain(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        motorRearLeft = hardwareMap.get(DcMotorEx.class, "rear_left");
        motorRearRight = hardwareMap.get(DcMotorEx.class, "rear_right");

        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);

        stop();
    }

    public void doDrive() {
        if (useGyroSupport) {
            calculateGyroSupport();
        }

        maxTargetSpeed = Math.max(Math.max(Math.max(Math.abs(motorFrontLeftTargetSpeed), Math.abs(motorFrontRightTargetSpeed)), Math.abs(motorRearLeftTargetSpeed)), Math.abs(motorRearRightTargetSpeed));

        if (maxTargetSpeed > 1.0) {
            motorFrontLeftTargetSpeed  /= maxTargetSpeed;
            motorFrontRightTargetSpeed /= maxTargetSpeed;
            motorRearLeftTargetSpeed   /= maxTargetSpeed;
            motorRearRightTargetSpeed  /= maxTargetSpeed;
        }

        motorFrontLeftTargetSpeed = correctPower(motorFrontLeftTargetSpeed);
        motorFrontRightTargetSpeed = correctPower(motorFrontRightTargetSpeed);
        motorRearLeftTargetSpeed = correctPower(motorRearLeftTargetSpeed);
        motorRearRightTargetSpeed = correctPower(motorRearRightTargetSpeed);

        if (useControlledAcceleration) {
            calculateControlledAcceleration();
        }

        if (!motorFrontLeftOnFire && Robot.errorDetection.detectFire("drive_front_left", motorFrontLeft.getVelocity(), motorFrontLeftTargetSpeed)) {
            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (!motorFrontRightOnFire && Robot.errorDetection.detectFire("drive_front_right", motorFrontRight.getVelocity(), motorFrontRightTargetSpeed)) {
            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (!motorRearLeftOnFire && Robot.errorDetection.detectFire("drive_rear_left", motorRearLeft.getVelocity(), motorRearLeftTargetSpeed)) {
            motorRearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (!motorRearRightOnFire && Robot.errorDetection.detectFire("drive_rear_right", motorRearRight.getVelocity(), motorRearRightTargetSpeed)) {
            motorRearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motorFrontLeft.setPower(motorFrontLeftTargetSpeed);
        motorFrontRight.setPower(motorFrontRightTargetSpeed);
        motorRearLeft.setPower(motorRearLeftTargetSpeed);
        motorRearRight.setPower(motorRearRightTargetSpeed);

        this.motorFrontLeftTargetSpeed = 0;
        this.motorFrontRightTargetSpeed = 0;
        this.motorRearLeftTargetSpeed = 0;
        this.motorRearRightTargetSpeed = 0;
    }

    public Drivetrain driveWithoutAnyEnhancements(double forwards, double sidewards, double turn) {
        return driveWithoutAnyEnhancements(forwards, sidewards, turn, 1, 1);
    }

    public Drivetrain driveWithoutAnyEnhancements(double forwards, double sidewards, double turn, double speedMultiplier, double sidewardsMultiplier) {
        this.forwards = forwards;
        this.sidewards = sidewards * Math.sqrt(2);
        this.turn = turn;

        double motorFrontLeftTargetSpeed = forwards + sidewards + turn;
        double motorFrontRightTargetSpeed = forwards - sidewards - turn;
        double motorRearLeftTargetSpeed = forwards - sidewards + turn;
        double motorRearRightTargetSpeed = forwards + sidewards - turn;

        maxTargetSpeed = Math.max(Math.max(Math.max(Math.abs(motorFrontLeftTargetSpeed), Math.abs(motorFrontRightTargetSpeed)), Math.abs(motorRearLeftTargetSpeed)), Math.abs(motorRearRightTargetSpeed));

        if (maxTargetSpeed > 1.0) {
            motorFrontLeftTargetSpeed  /= maxTargetSpeed;
            motorFrontRightTargetSpeed /= maxTargetSpeed;
            motorRearLeftTargetSpeed   /= maxTargetSpeed;
            motorRearRightTargetSpeed  /= maxTargetSpeed;
        }

        if (Math.abs(sidewards) > 0.3) {
            motorFrontLeftTargetSpeed  *= sidewardsMultiplier;
            motorFrontRightTargetSpeed *= sidewardsMultiplier;
            motorRearLeftTargetSpeed   *= sidewardsMultiplier;
            motorRearRightTargetSpeed  *= sidewardsMultiplier;
        } else {
            motorFrontLeftTargetSpeed  *= speedMultiplier;
            motorFrontRightTargetSpeed *= speedMultiplier;
            motorRearLeftTargetSpeed   *= speedMultiplier;
            motorRearRightTargetSpeed  *= speedMultiplier;
        }

        this.motorFrontLeftTargetSpeed += motorFrontLeftTargetSpeed;
        this.motorFrontRightTargetSpeed += motorFrontRightTargetSpeed;
        this.motorRearLeftTargetSpeed += motorRearLeftTargetSpeed;
        this.motorRearRightTargetSpeed += motorRearRightTargetSpeed;
        return this;
    }

    public Drivetrain setRotationOffset(double rotationOffset) {
        this.rotationOffset = rotationOffset;
        return this;
    }

    public Drivetrain drive(double forwards, double sidewards, double turn) {
        return drive(forwards, sidewards, turn, 1, 1);
    }

    public Drivetrain drive(double forwards, double sidewards, double turn, double speedMultiplier, double sidewardsMultiplier) {
        this.forwards = forwards;
        this.sidewards = sidewards;
        this.turn = turn;

        if (Robot.errorDetection.isMyMotorOnFire("dead_wheel_forwards") || Robot.errorDetection.isMyMotorOnFire("dead_wheel_sidewards")) {
            useDeadWheelSupport = false;
        }

        if (useHeadless) {
            calculateHeadless();
        }

        this.sidewards *= Math.sqrt(2);

        if (useHeadless && useDeadWheelSupport) {
            calculateDeadWheelSupport();
        }

        double motorFrontLeftTargetSpeed = this.forwards + this.sidewards + this.turn;
        double motorFrontRightTargetSpeed = this.forwards - this.sidewards - this.turn;
        double motorRearLeftTargetSpeed = this.forwards - this.sidewards + this.turn;
        double motorRearRightTargetSpeed = this.forwards + this.sidewards - this.turn;

        maxTargetSpeed = Math.max(Math.max(Math.max(Math.abs(motorFrontLeftTargetSpeed), Math.abs(motorFrontRightTargetSpeed)), Math.abs(motorRearLeftTargetSpeed)), Math.abs(motorRearRightTargetSpeed));

        if (maxTargetSpeed > 1.0) {
            motorFrontLeftTargetSpeed  /= maxTargetSpeed;
            motorFrontRightTargetSpeed /= maxTargetSpeed;
            motorRearLeftTargetSpeed   /= maxTargetSpeed;
            motorRearRightTargetSpeed  /= maxTargetSpeed;
        }

        if (Math.abs(sidewards) > 0.3) {
            motorFrontLeftTargetSpeed  *= sidewardsMultiplier;
            motorFrontRightTargetSpeed *= sidewardsMultiplier;
            motorRearLeftTargetSpeed   *= sidewardsMultiplier;
            motorRearRightTargetSpeed  *= sidewardsMultiplier;
        } else {
            motorFrontLeftTargetSpeed  *= speedMultiplier;
            motorFrontRightTargetSpeed *= speedMultiplier;
            motorRearLeftTargetSpeed   *= speedMultiplier;
            motorRearRightTargetSpeed  *= speedMultiplier;
        }

        this.motorFrontLeftTargetSpeed += motorFrontLeftTargetSpeed;
        this.motorFrontRightTargetSpeed += motorFrontRightTargetSpeed;
        this.motorRearLeftTargetSpeed += motorRearLeftTargetSpeed;
        this.motorRearRightTargetSpeed += motorRearRightTargetSpeed;
        return this;
    }

    public Drivetrain useHeadless(boolean useHeadless) {
        this.useHeadless = useHeadless;
        return this;
    }

    public Drivetrain useDriveMotorEncoderSupport(boolean useDriveMotorEncoderSupport) {
        this.useDriveMotorEncoderSupport = useDriveMotorEncoderSupport;
        if (useDriveMotorEncoderSupport) {
            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        return this;
    }

    public Drivetrain useGyroSupport(boolean useGyroSupport) {
        this.useGyroSupport = useGyroSupport;
        return this;
    }

    public Drivetrain useDeadWheelSupport(boolean useDeadWheelSupport) {
        this.useDeadWheelSupport = useDeadWheelSupport;
        return this;
    }

    public Drivetrain useControlledAcceleration(boolean useControlledAcceleration) {
        this.useControlledAcceleration = useControlledAcceleration;
        return this;
    }

    private void calculateHeadless() {
        double rotationRad = Robot.trackPosition.getRotationRad() - rotationOffset;
        double sin = Math.sin(rotationRad);
        double cos = Math.cos(rotationRad);

        double forwards = this.forwards;
        double sidewards = this.sidewards;

        this.forwards = forwards * cos + sidewards * sin;
        this.sidewards = -forwards * sin + sidewards * cos;
    }

    private void calculateGyroSupport() {
        pidTurn.setPIDGains(P,I,D);

        if (turn == 0) {
            double errorTurn = -Robot.trackPosition.getRotationSpeed();
            turn = pidTurn.calculatePIDAlgorithm(Math.abs(errorTurn)) * Math.signum(errorTurn);
            motorFrontLeftTargetSpeed += turn;
            motorFrontRightTargetSpeed -= turn;
            motorRearLeftTargetSpeed += turn;
            motorRearRightTargetSpeed -= turn;
        }
    }

    private void calculateControlledAcceleration() {
        double changeOfMotorFrontLeftSpeed = Math.abs(motorFrontLeftTargetSpeed) - Math.abs(lastMotorFrontLeftTargetSpeed);
        double changeOfMotorFrontRightSpeed = Math.abs(motorFrontRightTargetSpeed) - Math.abs(lastMotorFrontRightTargetSpeed);
        double changeOfMotorRearLeftSpeed = Math.abs(motorRearLeftTargetSpeed) - Math.abs(lastMotorRearLeftTargetSpeed);
        double changeOfMotorRearRightSpeed = Math.abs(motorRearRightTargetSpeed) - Math.abs(lastMotorRearRightTargetSpeed);

        double maxChangeOfMotorSpeed = Math.max(Math.max(Math.max(changeOfMotorFrontLeftSpeed, changeOfMotorFrontRightSpeed), changeOfMotorRearLeftSpeed), changeOfMotorRearRightSpeed);

        double maxChangeOfSpeedAllowed = (System.currentTimeMillis() - lastTimeLoop) / timeFromZeroToMaxSpeed;

        if (maxChangeOfMotorSpeed > maxChangeOfSpeedAllowed && maxChangeOfMotorSpeed != 0) {
            if (changeOfMotorFrontLeftSpeed > 0 && motorFrontLeftTargetSpeed != 0) {
                motorFrontLeftTargetSpeed = lastMotorFrontLeftTargetSpeed + Math.signum(motorFrontLeftTargetSpeed) * (changeOfMotorFrontLeftSpeed / (maxChangeOfMotorSpeed / maxChangeOfSpeedAllowed));
            }

            if (changeOfMotorFrontRightSpeed > 0 && motorFrontRightTargetSpeed != 0) {
                motorFrontRightTargetSpeed = lastMotorFrontRightTargetSpeed + Math.signum(motorFrontRightTargetSpeed) * (changeOfMotorFrontRightSpeed / (maxChangeOfMotorSpeed / maxChangeOfSpeedAllowed));
            }

            if (changeOfMotorRearLeftSpeed > 0 && motorRearLeftTargetSpeed != 0) {
                motorRearLeftTargetSpeed = lastMotorRearLeftTargetSpeed + Math.signum(motorRearLeftTargetSpeed) * (changeOfMotorRearLeftSpeed / (maxChangeOfMotorSpeed / maxChangeOfSpeedAllowed));
            }

            if (changeOfMotorRearRightSpeed > 0 && motorRearRightTargetSpeed != 0) {
                motorRearRightTargetSpeed = lastMotorRearRightTargetSpeed + Math.signum(motorRearRightTargetSpeed) * (changeOfMotorRearRightSpeed / (maxChangeOfMotorSpeed / maxChangeOfSpeedAllowed));
            }
        }

        lastMotorFrontLeftTargetSpeed = motorFrontLeftTargetSpeed;
        lastMotorFrontRightTargetSpeed = motorFrontRightTargetSpeed;
        lastMotorRearLeftTargetSpeed = motorRearLeftTargetSpeed;
        lastMotorRearRightTargetSpeed = motorRearRightTargetSpeed;

        lastTimeLoop = System.currentTimeMillis();
    }

    public void printStuff(Telemetry telemetry){
        telemetry.addData("forwards", forwards);
        telemetry.addData("sidewards", sidewards);
        telemetry.addData("turn", turn);

        telemetry.addData("error forwards", errorForward);
        telemetry.addData("error sidewards", errorSidewards);

        telemetry.addData("error", -Robot.trackPosition.getRotationSpeed());
        telemetry.addData("turn", turn * 100);
    }

    double errorForward = 0;
    double errorSidewards = 0;

    private void calculateDeadWheelSupport() {
        double forwardSpeed = Robot.trackPosition.getForwardSpeed();
        double sidewardsSpeed = Robot.trackPosition.getSidewardsSpeed();

        errorForward = 0;
        errorSidewards = 0;

        if (forwardSpeed != 0 && sidewards != 0) {
            errorForward = (sidewardsSpeed * forwards / (forwardSpeed * sidewards));
        }
        if (sidewardsSpeed != 0 && forwards != 0) {
            errorSidewards = (forwardSpeed * sidewards / (sidewardsSpeed * forwards));
        }

        double multiplierForward = pidForward.calculatePIDAlgorithm(errorForward * Math.signum(forwards));
        double multiplierSidewards = pidSidewards.calculatePIDAlgorithm(errorSidewards * Math.signum(sidewards));

        if (Math.abs(multiplierForward) < 1) {// || Math.abs(multiplierForward) > 2) {
            multiplierForward = Math.signum(forwards);
        }

        if (Math.abs(multiplierSidewards) < 1) {// || Math.abs(multiplierSidewards) > 2) {
            multiplierSidewards = Math.signum(sidewards);
        }

        forwards *= multiplierForward * Math.signum(forwards);
        sidewards *= multiplierSidewards * Math.signum(sidewards);
    }

    public void stop() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorRearLeft.setPower(0);
        motorRearRight.setPower(0);
    }

    double correctPower(double power) {
        if(Robot.controlHub.getInputVoltage(VoltageUnit.VOLTS) < 12) {
            minPowerNeededToTurn = 0.26;
        } else if (Robot.controlHub.getInputVoltage(VoltageUnit.VOLTS) < 10) {
            minPowerNeededToTurn = 0.28;
        } else if (Robot.controlHub.getInputVoltage(VoltageUnit.VOLTS) < 8) {
            minPowerNeededToTurn = 0.3;
        }
        if(power < minPowerNeededToTurn && power > -minPowerNeededToTurn) {
            return 0;
        }
        return power;
    }

    public Drivetrain setTimeFromZeroToMaxSpeed(double timeFromZeroToMaxSpeed) {
        this.timeFromZeroToMaxSpeed = timeFromZeroToMaxSpeed;
        return this;
    }

    public boolean getUseHeadless() {
        return useHeadless;
    }

    public boolean getUseDriveMotorEncoderSupport() {
        return useDriveMotorEncoderSupport;
    }

    public boolean getUseGyroSupport() {
        return useGyroSupport;
    }

    public boolean getUseDeadWheelSupport() {
        return useDeadWheelSupport;
    }
}
