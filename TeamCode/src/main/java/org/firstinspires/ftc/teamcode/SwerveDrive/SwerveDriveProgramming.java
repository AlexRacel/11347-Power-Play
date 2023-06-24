package org.firstinspires.ftc.teamcode.SwerveDrive;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveDriveProgramming {
    public DcMotorEx LeftFrontSwerveMotor;
    public DcMotorEx LeftBackSwerveMotor;
    public DcMotorEx RightFrontSwerveMotor;
    public DcMotorEx RightBackSwerveMotor;

    public BNO055IMU imu;
    private double offset = 0;
    private double robotAngle = 0;
    private double ANGLE_MARGIN_OF_ERROR = 5;
    private double powerFactor= 1;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public SwerveDriveProgramming(DcMotorEx m0, DcMotorEx m1, DcMotorEx m2, DcMotorEx m3, BNO055IMU imu1) {
        LeftFrontSwerveMotor = m0;
        LeftBackSwerveMotor = m1;
        RightFrontSwerveMotor = m2;
        RightBackSwerveMotor = m3;

        imu = imu1;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    Translation2d rightPod = new Translation2d(0, -0.115629);
    Translation2d leftPod = new Translation2d(0, 0.115629);
    SwerveDriveKinematics diffy = new SwerveDriveKinematics(rightPod, leftPod);

    ChassisSpeeds speed = new ChassisSpeeds(1.680972, 1.680972, 1.680972);
    SwerveModuleState[] moduleStates = diffy.toSwerveModuleStates(speed);

    // Front left module state
    SwerveModuleState right = moduleStates[0];

    // Front right module state
    SwerveModuleState left = moduleStates[1];
    SwerveModuleProgram r = new SwerveModuleProgram(RightFrontSwerveMotor, RightBackSwerveMotor);
    SwerveModuleProgram l = new SwerveModuleProgram(LeftFrontSwerveMotor, LeftBackSwerveMotor);
    public void drive(double y, double x, double rx) {
        robotAngle = -imu.getAngularOrientation().firstAngle;
        robotAngle = AngleUnit.normalizeRadians(robotAngle - offset);

        speed = new ChassisSpeeds(y, -x, -rx);
//        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
//                2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        moduleStates = diffy.toSwerveModuleStates(speed);
        l.updateEncoderPosition(RightFrontSwerveMotor.getCurrentPosition(), RightBackSwerveMotor.getCurrentPosition());
        r.updateEncoderPosition(LeftFrontSwerveMotor.getCurrentPosition(), LeftBackSwerveMotor.getCurrentPosition());
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.680972);
        ;
        RightFrontSwerveMotor.setVelocity(r.moveTo(right.speedMetersPerSecond, right.angle.getDegrees(), powerFactor));
        RightBackSwerveMotor.setVelocity(r.moveTo(right.speedMetersPerSecond, right.angle.getDegrees(), powerFactor));
        LeftBackSwerveMotor.setVelocity(l.moveTo(left.speedMetersPerSecond, left.angle.getDegrees(), powerFactor));
        LeftFrontSwerveMotor.setVelocity(l.moveTo(left.speedMetersPerSecond, left.angle.getDegrees(), powerFactor));
    }
}




