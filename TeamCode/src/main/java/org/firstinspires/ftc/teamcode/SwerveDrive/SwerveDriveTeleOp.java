
package org.firstinspires.ftc.teamcode.SwerveDrive;
        import com.arcrobotics.ftclib.hardware.motors.MotorEx;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp

public class SwerveDriveTeleOp extends LinearOpMode{
    SwerveDriveProgramming smp;
    SwerveModuleProgram smodp;
    public DcMotorEx LeftFrontSwerveMotor = null;
    public DcMotorEx LeftBackSwerveMotor = null;
    public DcMotorEx RightFrontSwerveMotor = null;
    public DcMotorEx RightBackSwerveMotor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        //Creates the gamepad object
        Gamepad p1 = new Gamepad();
        Gamepad c1 = new Gamepad();
        //defines the motors


        LeftFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fl");
        LeftFrontSwerveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontSwerveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFrontSwerveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontSwerveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        LeftFrontSwerveMotor.setPower(0);

        LeftBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "bl");
        LeftBackSwerveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackSwerveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackSwerveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontSwerveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        LeftBackSwerveMotor.setPower(0);

        RightFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fr");
        RightFrontSwerveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontSwerveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontSwerveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontSwerveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        RightFrontSwerveMotor.setPower(0);

        RightBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "bl");
        RightBackSwerveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackSwerveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackSwerveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackSwerveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        RightBackSwerveMotor.setPower(0);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        smp = new SwerveDriveProgramming(LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor, imu);

        Boolean pressed = false;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            smp.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
            telemetry.addLine("Swerve Test is Running");
            telemetry.update();
        }

    }
}
