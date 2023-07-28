package org.firstinspires.ftc.teamcode.SwerveDrive;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

public class SwerveDriveProgramming {
    public DcMotorEx
            LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor;
    public BNO055IMU imu;
    private double
            offset = 0, robotAngle = 0, ANGLE_MARGIN_OF_ERROR = 5, powerFactor = 1;
    SwerveModuleProgram r, l;
    public List<DcMotorEx> motors;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public SwerveDriveProgramming(List<DcMotorEx> motors, BNO055IMU imu1) {
        LeftFrontSwerveMotor = motors.get(0);
        LeftBackSwerveMotor = motors.get(1);;
        RightFrontSwerveMotor = motors.get(2);;
        RightBackSwerveMotor = motors.get(3);;

        this.motors = motors;

        imu = imu1;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        r = new SwerveModuleProgram(RightFrontSwerveMotor, RightBackSwerveMotor);
        l = new SwerveModuleProgram(LeftFrontSwerveMotor, LeftBackSwerveMotor);
    }

    Translation2d rightPod = new Translation2d(0, -0.115629), leftPod = new Translation2d(0, 0.115629);
    SwerveDriveKinematics diffy = new SwerveDriveKinematics(rightPod, leftPod);
    ChassisSpeeds speed = new ChassisSpeeds(1.680972, 1.680972, 1.680972);
    SwerveModuleState[] moduleStates = diffy.toSwerveModuleStates(speed);

    // Front left module state
    SwerveModuleState right = moduleStates[0], left = moduleStates[1];


    // Get radian angle for two measurements
    public void drive(double x, double y, double rx, double ry) {


        double headingOffset = 0;

        double kP = 0.5, kD = 0;

//        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - headingOffset);

//        Vector2d vec = new Vector2d(x, y).rotated(-heading); // Gives the angle that it needs to go.
        // For example, if your wanted position is up and left (1 each),
        // and your current robot heading is right and up ( 1 each )
        // The vector that this vec results in is pointing straight up (90 degrees)

//        double vecX = vec.getX();
//        double vecY = vec.getY();
//
//        double targetAngle = vec.angle();

        double magnitude = Math.max(rx + y, 1); // Caps at the amount

        // Simple Turning (The battle Begins)

        LeftFrontSwerveMotor.setPower((-rx + y) / magnitude);
        LeftBackSwerveMotor.setPower((-rx - y) / magnitude);

        RightFrontSwerveMotor.setPower((rx + y) / magnitude);
        RightBackSwerveMotor.setPower((rx - y) / magnitude);


//        forward --> m1 + m2 -;
//        backward --> m1 - m2 +;
//        cw rotation --> m1 + m2 +;
//        ccw rotation --> m1 - m2 -;


//        moduleStates = diffy.toSwerveModuleStates(new ChassisSpeeds(y, -x, -rx));
//
//        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.680972);

//        return new double [][] {
//                r.moduleController(right.speedMetersPerSecond, right.angle.getDegrees(), powerFactor),
//                l.moduleController(left.speedMetersPerSecond, left.angle.getDegrees(), powerFactor)
//        };

    }
}




