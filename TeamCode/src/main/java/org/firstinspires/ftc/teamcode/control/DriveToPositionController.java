package org.firstinspires.ftc.teamcode.control;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.MatchData;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.Position;

@Config 
@SuppressWarnings({"unused"})
public class DriveToPositionController {
    final PIDController pidForwardsDrive = new PIDController(0.002, 0, 0.0005);
    final PIDController pidSidewardsDrive = new PIDController(0.002, 0, 0.0005);
    final PIDController pidTurnDrive = new PIDController(0.008, 0, 0.001);

    final PIDController pidForwardsHold = new PIDController(0.02, 0.001, 0.005);
    final PIDController pidSidewardsHold = new PIDController(0.02, 0.001, 0.005);
    final PIDController pidTurnHold = new PIDController(0.04, 0.002, 0.01);

    PIDController pidForwards = pidForwardsDrive; // All PID values are currently under tests to be optimised
    PIDController pidSidewards = pidSidewardsDrive;
    PIDController pidTurn = pidTurnDrive;
    double speedForwards = 0;
    double speedSidewards = 0;

    public double forwards = 0;
    public double sidewards = 0;
    public double turn = 0;
    Position maxDeviation = new Position(20.0, 20.0, 10.0);
    Position currentPosition = new Position(0.0, 0.0, 0.0);
    Position targetPosition = new Position(null, null, null);

    boolean activated = false;

    final int reverseIfBlueAlliance = MatchData.redAlliance ? 1 : -1;

    double minMotorPower = 0.25;
    double maxMotorPower = 0.8;

    public boolean firstLoopAfterSettingDone = false;
    boolean permissionToCheckLoop = true;
    boolean checkLoop = false;

    Double angleOfMovement = null;

    final PIDController pidPathing = new PIDController(0.002, 0, 0.0005);
    final PIDController pidPathingTurn = new PIDController(0.008, 0, 0.001);
    final PIDController pidCorrectiveX = new PIDController(0.02, 0.001, 0.005);
    final PIDController pidCorrectiveY = new PIDController(0.02, 0.001, 0.005);
    final PIDController pidCorrectiveTurn = new PIDController(0.04, 0.002, 0.01);

    Runnable driveToPosition = () -> {
        while (Robot.state != Robot.State.SHOULD_STOP && activated) {
            //pidForwards.setPIDGains(minMotorPower / maxDeviation.y, 0, D_FORWARDS * minMotorPower); // All PID values are currently under tests to be optimised
            //pidSidewards.setPIDGains(minMotorPower / maxDeviation.x, 0, D_SIDEWARDS * minMotorPower);
            //pidTurn.setPIDGains(minMotorPower / maxDeviation.rotation, 0, D_TURN * minMotorPower);

            if (permissionToCheckLoop) {
                permissionToCheckLoop = false;
                checkLoop = true;
            }
            currentPosition = Robot.trackPosition.getPosition();
            double rotationRad = Robot.trackPosition.getRotationRad();
            double sin = Math.sin(rotationRad);
            double cos = Math.cos(rotationRad);

            double xNeededToTargetPosition = 0;
            if (targetPosition.x != null) {
                xNeededToTargetPosition = targetPosition.x * reverseIfBlueAlliance - currentPosition.x;
            }

            double yNeededToTargetPosition = 0;
            if (targetPosition.y != null) {
                yNeededToTargetPosition = targetPosition.y - currentPosition.y;
            }


            if (xNeededToTargetPosition != 0 || yNeededToTargetPosition != 0) {
                forwards = correctPower(pidForwards.calculatePIDAlgorithm(yNeededToTargetPosition * cos + xNeededToTargetPosition * sin));
                sidewards = correctPower(pidSidewards.calculatePIDAlgorithm(-yNeededToTargetPosition * sin + xNeededToTargetPosition * cos));
                speedForwards = forwards;
                speedSidewards = sidewards;
            }


            if (targetPosition.rotation != null) {
                double rotationNeededToTargetPosition = targetPosition.rotation * reverseIfBlueAlliance - currentPosition.rotation;
                turn = correctPower(pidTurn.calculatePIDAlgorithm(rotationNeededToTargetPosition));
            }

            if (checkLoop) {
                firstLoopAfterSettingDone = true;
                checkLoop = false;
            }

            if (activated) {
                Robot.drivetrain.driveWithoutAnyEnhancements(forwards, sidewards, turn).doDrive();
            }
            if (!activated) {
                Robot.drivetrain.stop();
            }
        }
    };

    Runnable newDriveToPositionController = () -> {
        while (Robot.state != Robot.State.SHOULD_STOP && activated) {
            //TODO: This is just a concept it needs a lot of more work and testing
            if (permissionToCheckLoop) {
                permissionToCheckLoop = false;
                checkLoop = true;
            }

            currentPosition = Robot.trackPosition.getPosition();

            // X and Y calculation

            double powerXPathing = 0;
            double powerYPathing = 0;

            double powerYCorrective = 0;
            double powerXCorrective = 0;

            double totalDistancePathing = 0;

            double distanceX = 0;
            if (targetPosition.x != null) {
                distanceX = targetPosition.x - currentPosition.x;
            }

            double distanceY = 0;
            if (targetPosition.y != null) {
                distanceY = targetPosition.y - currentPosition.y;
            }

            if (angleOfMovement != null && targetPosition.y != null && targetPosition.x != null) {
                if (angleOfMovement != 0 && angleOfMovement != PI) {
                    double totalDistanceAccordingToX = distanceX / cos(angleOfMovement);
                    double totalDistanceAccordingToY = distanceY / sin(angleOfMovement);
                    totalDistancePathing = Math.min(totalDistanceAccordingToX, totalDistanceAccordingToY);

                    double powerRelative = pidPathing.calculatePIDAlgorithm(totalDistancePathing);

                    powerXPathing = sin(angleOfMovement) * powerRelative;
                    powerYPathing = cos(angleOfMovement) * powerRelative;

                    if (totalDistanceAccordingToX < totalDistanceAccordingToY) {
                        double totalDistanceCorrectiveY = distanceY - distanceX * tan(angleOfMovement);
                        powerYCorrective = pidCorrectiveY.calculatePIDAlgorithm(totalDistanceCorrectiveY);
                    } else {
                        double totalDistanceCorrectiveX = distanceX - tan(angleOfMovement) / distanceY;
                        powerXCorrective = pidCorrectiveX.calculatePIDAlgorithm(totalDistanceCorrectiveX);
                    }
                } else if (angleOfMovement == 0) {
                    totalDistancePathing = distanceX;
                    powerXPathing = pidPathing.calculatePIDAlgorithm(distanceX);
                    powerYCorrective = pidCorrectiveY.calculatePIDAlgorithm(distanceY);
                } else {
                    totalDistancePathing = distanceY;
                    powerYPathing = pidPathing.calculatePIDAlgorithm(distanceY);
                    powerXCorrective = pidCorrectiveX.calculatePIDAlgorithm(distanceX);
                }
            } else {
                if (targetPosition.y != null) {
                    powerYCorrective = pidCorrectiveY.calculatePIDAlgorithm(distanceY);
                }
                if (targetPosition.x != null) {
                    powerXCorrective = pidCorrectiveX.calculatePIDAlgorithm(distanceX);
                }
            }

            double powerXTotal = powerXPathing + powerXCorrective;
            double powerYTotal = powerYPathing + powerYCorrective;

            double rotationRad = toRadians(currentPosition.rotation);
            double sin = Math.sin(rotationRad);
            double cos = Math.cos(rotationRad);

            double forwards = powerYTotal * cos + powerXTotal * sin;
            double sidewards = -powerYTotal * sin + powerXTotal * cos;

            // Rotation calculation

            double turn = 0;
            if (targetPosition.rotation != null) {
                double distanceTurn = targetPosition.rotation - currentPosition.rotation;

                if (distanceTurn > maxDeviation.rotation || distanceTurn < -maxDeviation.rotation) {
                    turn = pidPathingTurn.calculatePIDAlgorithm(distanceTurn);
                } else {
                    turn = pidCorrectiveTurn.calculatePIDAlgorithm(distanceTurn);
                }
            }

            if (checkLoop) {
                firstLoopAfterSettingDone = true;
                checkLoop = false;
            }

            // Velocity Based Stopping (Concept)
            /*
            double deceleration = 10;
            double currentVelocity = Math.sqrt(Math.pow(Robot.trackPosition.getForwardSpeed(), 2) + Math.pow(Robot.trackPosition.getSidewardsSpeed(), 2));
            double changeInPosition = ((currentVelocity * currentVelocity) / (2 * deceleration)) * signum(currentVelocity);

            double changeInY = sin(angleOfMovement) * changeInPosition;
            double changeInX = cos(angleOfMovement) * changeInPosition;

            double estimatedX = currentPosition.x + changeInX;
            double estimatedY = currentPosition.y + changeInY;

            double velocity = 100;

            if (changeInPosition > totalDistancePathing) {
                // Switch to velocity based stopping

                // Velocity based stopping:
                velocity -= deceleration;
            }
             */

            if (activated) {
                Robot.drivetrain.driveWithoutAnyEnhancements(forwards, sidewards, turn).doDrive();
            }
            if (!activated) {
                Robot.drivetrain.stop();
            }
        }
    };

    public void setMaxDeviation(double maxDeviationX, double maxDeviationY, double maxDeviationRotation) {
        maxDeviation.set(maxDeviationX, maxDeviationY, maxDeviationRotation);
    }

    public Position getTargetPosition() {
        return targetPosition;
    }

    public DriveToPositionController setTargetPosition(Double targetPosX, Double targetPosY, Double targetRotation) {

        /*
        if (targetRotation != null) {
            double rotation90 = targetRotation % 180;
        }
        if ((targetPosY < targetPosition.y + 20 && targetPosY > targetPosition.y - 20 && (targetRotation % 180) == 0)) {
            pidForwards = pidForwardsHold;
        } else {
            pidForwards = pidForwardsDrive;
        }
        if (targetPosX < targetPosition.x + 20 && targetPosX > targetPosition.x - 20) {
            pidSidewards = pidSidewardsHold;
        } else {
            pidSidewards = pidSidewardsDrive;
        }

        if (targetRotation < targetPosition.rotation + 20 && targetRotation > targetPosition.rotation - 20) {
            pidTurn = pidTurnHold;
        } else {
            pidTurn = pidTurnDrive;
        }
        */

        firstLoopAfterSettingDone = false;
        targetPosition = new Position(targetPosX, targetPosY, targetRotation);
        permissionToCheckLoop = true;
        return this;
    }

    public DriveToPositionController newSetTargetPosition(Position targetPosition) {
        double differenceY = this.targetPosition.y - targetPosition.y;
        double differenceX = this.targetPosition.x - targetPosition.x;

        firstLoopAfterSettingDone = false;

        if (differenceX != 0) {
            angleOfMovement = atan(differenceY / differenceX);
        } else if (differenceY != 0) {
            angleOfMovement = PI;
        } else {
            angleOfMovement = null;
        }
        this.targetPosition = targetPosition;

        permissionToCheckLoop = true;
        return this;
    }

    public DriveToPositionController setTargetPosition(Position targetPosition) {

        /*
        if (Objects.equals(this.targetPosition.y, targetPosition.y) && targetPosition.y != null) {
            pidForwards = pidForwardsHold;
        } else {
            pidForwards = pidForwardsDrive;
        }

        if (Objects.equals(this.targetPosition.x, targetPosition.x) && targetPosition.y != null) {
            pidSidewards = pidSidewardsHold;
        } else {
            pidSidewards = pidSidewardsDrive;
        }

        if (Objects.equals(this.targetPosition.rotation, targetPosition.rotation) && targetPosition.rotation != null) {
            pidTurn = pidTurnHold;
        } else {
            pidTurn = pidTurnDrive;
        }
        */

        firstLoopAfterSettingDone = false;
        this.targetPosition = targetPosition;
        permissionToCheckLoop = true;
        return this;
    }

    double correctPower(double power) {
        double voltage = Robot.controlHub.getInputVoltage(VoltageUnit.VOLTS);
        /*minMotorPower = 0.25;
        if(voltage < 12) {
            minMotorPower = 0.26;
        } else if (voltage < 10) {
            minMotorPower = 0.28;
        } else if (voltage < 8) {
            minMotorPower = 0.3;
        }*/
        minMotorPower = 0.4 * (Math.pow(0.96, voltage));
        if(power < 0.05 && power > -0.05) {
            return 0;
        }
        if(power > 0) {
            power = minMotorPower + power * (1 - minMotorPower);
        } else {
            power = -minMotorPower + power * (1 - minMotorPower);
        }
        
        if(power > maxMotorPower){
            return maxMotorPower;
        } else if (power < -maxMotorPower) {
            return -maxMotorPower;
        }
        return power;
    }

    class Error {
        public double AverageOfDistance() {
            double errorX = abs(targetPosition.x - currentPosition.x);
            double errorY = abs(targetPosition.y - currentPosition.y);
            return (errorX + errorY) / 2;
        }

        public double rotation() {
            return targetPosition.rotation - currentPosition.rotation;
        }

        public double x() {
            return targetPosition.x - currentPosition.x;
        }

        public double y() {
            return targetPosition.y - currentPosition.y;
        }

        public void log(boolean include_average, boolean auto_update, Telemetry telemetry) {
            telemetry.addLine("____________________");
            telemetry.addLine("| Errors of DriveToPositionController");
            telemetry.addData("|    Rotation: ", rotation());
            telemetry.addData("|    X: ", x());
            telemetry.addData("|    Y: ", y());
            if (include_average) {telemetry.addData("|  Average of distance: ", AverageOfDistance());}
            telemetry.addLine("____________________");
            if (auto_update) {telemetry.update();}
        }
    }

    public boolean isPositionReached() {
        Position currentPosition = this.currentPosition;
        Position targetPosition = this.targetPosition;

        if (currentPosition.x == null || currentPosition.y == null || currentPosition.rotation == null) {
            currentPosition = Robot.trackPosition.getPosition();
        }

        double sin = Math.sin(currentPosition.rotation);
        double cos = Math.cos(currentPosition.rotation);

        boolean xReached = true;
        if (targetPosition.x != null) {
            xReached = (targetPosition.x * reverseIfBlueAlliance - maxDeviation.x < currentPosition.x && targetPosition.x * reverseIfBlueAlliance + maxDeviation.x > currentPosition.x) || (sidewards * cos + forwards * sin  == 0 && firstLoopAfterSettingDone);
        }

        boolean yReached = true;
        if (targetPosition.y != null) {
            yReached = (targetPosition.y < currentPosition.y + maxDeviation.y && targetPosition.y > currentPosition.y - maxDeviation.y) || (forwards * cos - sidewards * sin == 0 && firstLoopAfterSettingDone);
        }

        boolean rotationReached = true;
        if (targetPosition.rotation != null) {
            rotationReached = (targetPosition.rotation * reverseIfBlueAlliance < currentPosition.rotation + maxDeviation.rotation && targetPosition.rotation * reverseIfBlueAlliance > currentPosition.rotation - maxDeviation.rotation) || (turn == 0 && firstLoopAfterSettingDone);
        }

        return xReached && yReached && rotationReached;
    }

    public void waitUntilPositionIsReached(int timeoutMs) {
        long targetMillis = System.currentTimeMillis() + timeoutMs;
        boolean timeoutReached = false;
        while (!isPositionReached() && !timeoutReached && Robot.state != Robot.State.SHOULD_STOP) {
            if (targetMillis < System.currentTimeMillis()) {
                timeoutReached = true;
                Robot.errorDetection.addErrorToErrorLog("drive to position ended due to timeout");
            }
        }
    }

    public void activate() {
        activated = true;
        Robot.addExecutor(driveToPosition);
    }

    public void deactivate() {
        activated = false;
        Robot.drivetrain.stop();
    }

    public void setMaxMotorPower(double maxMotorPower) {
        this.maxMotorPower = maxMotorPower;
    }
}
