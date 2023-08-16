package org.firstinspires.ftc.teamcode.SwerveDrive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Module {
    private double
            rPower, oppositeAngle, angleToTarget, oppAngleFromTarget,
            moduleAngle = 0, currentAngleTicks = 0,
            tuner = 2,
            ANGLE_MARGIN_OF_ERROR = 5,
            bigGearRatio = 23/68,
            ticksPerRev = 145.1,
            rotationsPerSec = 1150/60,
            MAX_RPS_TICKS = rotationsPerSec * ticksPerRev, // 2781
            TICKS_PER_DEGREE_BIG_GEAR = MAX_RPS_TICKS * bigGearRatio;

    private int
            topPosition = 0, bottomPosition = 0,
            topRotationPosition = 0,
            bottomRotationPosition = 0,
            topOffsetDirectional = 0,
            bottomOffsetDirectional = 0;

    private DcMotorEx
            topModule, bottomModule;
    private PID sPID = new PID(0.0184, 0.3, 0.02);

    Telemetry telemetry;

    public Module(DcMotorEx top, DcMotorEx bottom) {
        topModule = top;
        bottomModule = bottom;
    }
    public void updateEncoderPosition(){
        // Get encoder positions in degrees
        topPosition = topModule.getCurrentPosition();
        bottomPosition = bottomModule.getCurrentPosition();
    }

    public double getAngle() {
        // Since any forward movement ADDS and SUBTRACTS even amounts
        // from both the top and bottom (top + 150) and (bottom - 150),
        // the average SHOULD work
        currentAngleTicks = (topPosition + bottomPosition) / 2.0;
        moduleAngle = currentAngleTicks / TICKS_PER_DEGREE_BIG_GEAR;
        moduleAngle = angleWrap(moduleAngle);
        return moduleAngle;
    }

    public void moduleController(double velocity, double targetAngle, double power) {
        updateEncoderPosition();

        // Get both angles for the module itself in deg.
        moduleAngle = getAngle();
        oppositeAngle = angleWrap(moduleAngle + 180);

        // Gets what rotation of how many angles to target
        angleToTarget = angleWrap(targetAngle - moduleAngle);
        oppAngleFromTarget = angleWrap(targetAngle - oppositeAngle);

        // If the angle on the opposite side is closer than the current target,
        // we make that the current target to avoid sus turning
        if (Math.abs(angleToTarget) > Math.abs(oppAngleFromTarget)) {
            // Set the rotational power based on the PID for the angle
            rPower = sPID.PIDControl(moduleAngle,oppAngleFromTarget);
        }
        else {
            // Set the rotational power based on the PID for the angle
            rPower = sPID.PIDControl(moduleAngle, angleToTarget);
        }

        if (Math.abs(oppAngleFromTarget) < ANGLE_MARGIN_OF_ERROR) {
            velocity = -velocity;
        }
        else if (Math.abs(angleToTarget) > ANGLE_MARGIN_OF_ERROR) {
            velocity = 0;
        }

//        // Ensure you never exceed power limits [-1, 1]
//        double magnitude;
//        double topMagnitude = Math.max((velocity * power) + rPower, 1);
//        double bottomMagnitude = Math.max((-velocity * power) + rPower, 1);
//        if (topMagnitude > bottomMagnitude) {
//            magnitude = topMagnitude;
//        }
//        else {
//            magnitude = bottomMagnitude;
//        }

        topModule.setVelocity(((velocity * power) + rPower) * (MAX_RPS_TICKS/tuner));
        bottomModule.setVelocity(((-velocity * power) + rPower) * (MAX_RPS_TICKS/tuner));
//
//        // Do the thing
//        //TODO May need to change this to velocity later if this doesn't work
//        topModule.setPower(
//                ( (velocity * power) + rPower ) / magnitude
//        );
//        bottomModule.setPower(
//                ( (-velocity * power) + rPower ) / magnitude
//        );
    }


}





