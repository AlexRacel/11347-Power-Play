package org.firstinspires.ftc.teamcode.SwerveDrive;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    // Front right module state

    public void drive(double x, double y, double rx) {
        double headingOffset = 0;

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - headingOffset);

//        robotAngle = AngleUnit.normalizeRadians(
//                (-imu.getAngularOrientation().firstAngle)
//        );

        double magnitude = Math.sqrt(y*y + x*x);
        double motor1power = 0, motor2power = 0;

        double tentiarey = 1200;

//        motor1power += rx;
//        motor2power += rx;

        LeftFrontSwerveMotor.setVelocity(tentiarey*(-rx + y));
        LeftBackSwerveMotor.setVelocity(tentiarey*(-rx - y));

        RightFrontSwerveMotor.setVelocity(tentiarey*(rx + y));
        RightBackSwerveMotor.setVelocity(tentiarey*(rx + y));



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




