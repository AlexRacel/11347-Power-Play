package org.firstinspires.ftc.teamcode.SwerveDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class SwerveDriveProgramming {
    public DcMotorEx
            LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor;
    public BNO055IMU imu;
    private double
            offset = 0, robotAngle = 0, ANGLE_MARGIN_OF_ERROR = 5, powerFactor = 1;
    SwerveModuleProgram r, l;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private Telemetry telemetry;

    public SwerveDriveProgramming(List<DcMotorEx> motors, BNO055IMU imu1, Telemetry telemetry) {
        LeftFrontSwerveMotor = motors.get(0);
        LeftBackSwerveMotor = motors.get(1);;
        RightFrontSwerveMotor = motors.get(2);;
        RightBackSwerveMotor = motors.get(3);;

        imu = imu1;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        this.telemetry = telemetry;

        r = new SwerveModuleProgram(RightFrontSwerveMotor, RightBackSwerveMotor, telemetry);
        l = new SwerveModuleProgram(LeftFrontSwerveMotor, LeftBackSwerveMotor, telemetry);
    }

    Translation2d rightPod = new Translation2d(0, -0.115629), leftPod = new Translation2d(0, 0.115629);
    SwerveDriveKinematics diffy = new SwerveDriveKinematics(rightPod, leftPod);

    ChassisSpeeds speed = new ChassisSpeeds(1.680972, 1.680972, 1.680972);
    SwerveModuleState[] moduleStates = diffy.toSwerveModuleStates(speed);

    // Front left module state
    SwerveModuleState right = moduleStates[0], left = moduleStates[1];

    // Front right module state

    public void drive(double y, double x, double rx) {
        robotAngle = AngleUnit.normalizeRadians(
                (-imu.getAngularOrientation().firstAngle) - offset
        );


//        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
//                2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        moduleStates = diffy.toSwerveModuleStates(new ChassisSpeeds(y, -x, -rx));

        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.680972);

        r.moveTo(right.speedMetersPerSecond, right.angle.getDegrees(), powerFactor);
        l.moveTo(left.speedMetersPerSecond, left.angle.getDegrees(), powerFactor);
    }
}




