
package org.firstinspires.ftc.teamcode.SwerveDrive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@TeleOp (name="Alex goes sideways")
public class SwerveDriveTeleOp extends CommandOpMode {
    SwerveDriveProgramming swerveDrive;

    public DcMotorEx
            LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor;
    BNO055IMU imu;

    @Override
    public void initialize() {
//        Initialize the stuff here otherwise things can get illegal, fast
        LeftFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fl");
        LeftBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "bl");
        RightFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fr");
        RightBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "br");


//        Instead of changing every single motor, we set all their properties at once through a list.
        List<DcMotorEx> motors = Arrays.asList(LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor);
        for (DcMotorEx motor : motors) {
//            Anything in here will get repeated for each individual "motor" in the list
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

//        Initialize the other stuff here now
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        swerveDrive = new SwerveDriveProgramming(motors, imu);

        telemetry.addLine("Initialized");
    }
    @Override
    public void run() {
        super.run();

        double[][] observations = swerveDrive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        for (double[] moveToObject : observations) {
            telemetry.addLine(String.format("%f %f", moveToObject[0], moveToObject[1]));
        }

        telemetry.addLine(String.format("%f %f %f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
        telemetry.addLine("Swerve Test is Running");
        telemetry.update();

    }
}
