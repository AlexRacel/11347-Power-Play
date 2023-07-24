package org.firstinspires.ftc.teamcode.SwerveDrive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SwerveDrive.SwervePID;

public class SwerveModuleProgram {
    private double moduleAngle = 0;
    private double ANGLE_MARGIN_OF_ERROR =5;
    private DcMotorEx frontmod;
    private DcMotorEx backmod;
    private double rPower;
    private double oppAngle;
    private double angleFromTarget;
    private double oppAngleFromTarget;
    private double smallGearRatio = 63/18;
    private double bigGearRatio = 23/68;
    private double ticksPerRev = 145.1;
    private double rotationsPerSec = 1150/60;
    private double MAX_RPS_TICKS = rotationsPerSec * ticksPerRev;
    private double TICKS_PER_DEGREE_BIG_GEAR = MAX_RPS_TICKS * bigGearRatio;
    private double currentAngleTicks = 0;
    private double currentPositionTicks = 0;
    private double currentPosition = 0;
    private double TICKS_PER_INCH = TICKS_PER_DEGREE_BIG_GEAR * smallGearRatio * Math.PI * 2.5;
    private int frontPosition = 0;
    private int backPosition = 0;
    SwervePID PIDr = new SwervePID(1, 0, 0);
    public SwerveModuleProgram(DcMotorEx front, DcMotorEx back) {
        frontmod = front;
        backmod = back;

    }
    public void updateEncoderPosition(int frontPosition, int backPosition){
        this.frontPosition = frontPosition;
        this.backPosition = backPosition;

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

        frontmod.setVelocity(((-velocity * power) + rPower) * MAX_RPS_TICKS);
        backmod.setVelocity(((velocity * power) + rPower) * MAX_RPS_TICKS);
    }


}





