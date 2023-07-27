package org.firstinspires.ftc.teamcode.SwerveDrive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveModuleProgram {
    private double
            moduleAngle = 0, ANGLE_MARGIN_OF_ERROR = 5, rPower, oppAngle, angleFromTarget,
            oppAngleFromTarget, smallGearRatio = 63/18, bigGearRatio = 23/68, ticksPerRev = 145.1,
            rotationsPerSec = 1150/60, MAX_RPS_TICKS = rotationsPerSec * ticksPerRev,
            TICKS_PER_DEGREE_BIG_GEAR = MAX_RPS_TICKS * bigGearRatio, currentAngleTicks = 0,
            currentPositionTicks = 0, currentPosition = 0,
            TICKS_PER_INCH = TICKS_PER_DEGREE_BIG_GEAR * smallGearRatio * Math.PI * 2.5;
    private int
            frontPosition = 0, backPosition = 0;

//    Finals are simple, it just means you only set it once. Since you don't change the module, this works!
    private final DcMotorEx
            frontModule, backModule;
    SwervePID PIDr = new SwervePID(1, 0, 0);

    Telemetry telemetry;

    public SwerveModuleProgram(DcMotorEx front, DcMotorEx back, Telemetry teleImport) {
        frontModule = front;
        backModule = back;

        telemetry = teleImport;
    }
    public void updateEncoderPosition(){
        frontPosition = frontModule.getCurrentPosition();
        backPosition = backModule.getCurrentPosition();

    }
    public double getWheelPositionInch() {
        currentPositionTicks = (frontPosition - backPosition)/2.0;
        currentPosition = currentPositionTicks / TICKS_PER_INCH;
        return currentPosition;
    }
    public double getAngle() {
        currentAngleTicks = (frontPosition + backPosition) / 2.0;
        moduleAngle = currentAngleTicks / TICKS_PER_DEGREE_BIG_GEAR;
        moduleAngle = angleWrap(moduleAngle);
        return moduleAngle;
    }
    public void moveTo(double velocity, double targetAngle, double power) {

        moduleAngle = getAngle();
        oppAngle = angleWrap(moduleAngle + 180);

        angleFromTarget = angleWrap(targetAngle - moduleAngle);
        oppAngleFromTarget = angleWrap(targetAngle - oppAngle);

        if (Math.abs(angleFromTarget) > Math.abs(oppAngleFromTarget)) {
            rPower = PIDr.PIDControl(moduleAngle,oppAngleFromTarget);
        }
        else {
            rPower = PIDr.PIDControl(moduleAngle, angleFromTarget);
        }

        if (Math.abs(oppAngleFromTarget) < ANGLE_MARGIN_OF_ERROR) {
            velocity = -velocity;
        }
        else if (Math.abs(angleFromTarget) > ANGLE_MARGIN_OF_ERROR) {
            velocity = 0;
        }

        frontModule.setVelocity(((-velocity * power) + rPower) * MAX_RPS_TICKS);
        backModule.setVelocity(((velocity * power) + rPower) * MAX_RPS_TICKS);

        telemetry.addData("Front Module Velocity: ", frontModule.getVelocity());
        telemetry.addData("Back Module Velocity: ", backModule.getVelocity());
    }


}





