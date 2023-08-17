package org.firstinspires.ftc.teamcode.SwerveDrive;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

public class Coordinator {
    public DcMotorEx
            LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor;
    public BNO055IMU imu;
    Module r, l;
    public List<DcMotorEx> motors;

    // Increase or decrease this to increase overall speed,
    // be warned that too much gain gives too little to rotation as
    // it is normalized to fit [-1, 1]
    public double gain = 1;

    Translation2d rightPod, leftPod;
    SwerveDriveKinematics diffy;
    ChassisSpeeds speed;
    SwerveModuleState[] moduleStates;
    SwerveModuleState right, left;
    BNO055IMU.Parameters parameters;
    Telemetry telemetry;

    public Coordinator(Telemetry telemetry, List<DcMotorEx> motors, BNO055IMU imu1) {
        this.telemetry = telemetry;
        LeftFrontSwerveMotor = motors.get(0);
        LeftBackSwerveMotor = motors.get(1);;
        RightFrontSwerveMotor = motors.get(2);;
        RightBackSwerveMotor = motors.get(3);;

        this.motors = motors;

        // Initializing everything in here just in case
        rightPod = new Translation2d(0, -0.115629);
        leftPod = new Translation2d(0, 0.115629);
        diffy = new SwerveDriveKinematics(rightPod, leftPod);
        parameters = new BNO055IMU.Parameters();

        imu = imu1;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        r = new Module(RightFrontSwerveMotor, RightBackSwerveMotor);
        l = new Module(LeftFrontSwerveMotor, LeftBackSwerveMotor);
    }


    // Get radian angle for two measurements
    public void drive(double x, double y, double rx, double ry) {
//        speed = new ChassisSpeeds(1.680972, 1.680972, 1.680972);
        speed = new ChassisSpeeds(x, y, rx);
        moduleStates = diffy.toSwerveModuleStates(speed);
        right = moduleStates[0];
        left = moduleStates[1];
        double headingOffset = 0;

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - headingOffset);

//        Vector2d vec = new Vector2d(x, y).rotated(-heading); // Gives the angle that it needs to go.
        // For example, if your wanted position is up and left (1 each),
        // and your current robot heading is right and up ( 1 each )
        // The vector that this vec results in is pointing straight up (90 degrees)

//        double vecX = vec.getX(); // How much to move left or right
//        double vecY = vec.getY(); // How much to move forward, or backward
//
//        double targetAngle = vec.angle();
//        rx = rx * 0.25;
////
//        double magnitude = Math.max(Math.abs(rx) + Math.abs(y), 1); // Caps at the amount, works for rx+y, -rx-y

        // Simple Turning (The battle Begins)
//        LeftFrontSwerveMotor.setPower((-rx + y) / magnitude);
//        LeftBackSwerveMotor.setPower((-rx - y) / magnitude);
//
//        RightFrontSwerveMotor.setPower((rx - y) / magnitude);
//        RightBackSwerveMotor.setPower((rx + y) / magnitude);


//        forward --> m1 + m2 -;
//        backward --> m1 - m2 +;
//        cw rotation --> m1 + m2 +;
//        ccw rotation --> m1 - m2 -;


        // Make sure your speeds are correct values
        moduleStates = diffy.toSwerveModuleStates(new ChassisSpeeds(y, x, -rx));
//
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.680972);

        r.moduleController(right.speedMetersPerSecond, right.angle.getDegrees(), gain);
        l.moduleController(left.speedMetersPerSecond, -left.angle.getDegrees(), gain);

        Module[] listOfModules = new Module[]{r, l};
        telemetry.addData("angle", right.angle.getDegrees());
        telemetry.addData("angleL", left.angle.getDegrees());
        telemetry.addData("mpsl", left.speedMetersPerSecond);
        telemetry.addData("mpsr", right.speedMetersPerSecond);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);


        for (Module mod: listOfModules){
            telemetry.addData("angle", mod.getAngle());
            telemetry.addData("TM", mod.topModuleVelocity);
            telemetry.addData("BM", mod.bottomModuleVelocity);
            telemetry.addData("ToTarget", mod.angleToTarget);
        }
    }
}




