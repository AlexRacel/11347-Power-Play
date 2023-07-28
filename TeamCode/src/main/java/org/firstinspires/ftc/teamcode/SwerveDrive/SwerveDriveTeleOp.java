
package org.firstinspires.ftc.teamcode.SwerveDrive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;


@TeleOp (name="Alex IS sideways (Demo)")
public class SwerveDriveTeleOp extends CommandOpMode {
    SwerveDriveProgramming swerveDrive;

    public DcMotorEx
            LeftFrontSwerveMotor, LeftBackSwerveMotor, RightFrontSwerveMotor, RightBackSwerveMotor;
    BNO055IMU imu;
    List<LynxModule> allHubs;

    @Override
    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


//        Initialize the stuff here otherwise things can get illegal, fast
        LeftFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fl");
        LeftBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "bl");
        RightFrontSwerveMotor = hardwareMap.get(DcMotorEx.class, "fr");
        RightBackSwerveMotor = hardwareMap.get(DcMotorEx.class, "br");

//        Instead of changing every single motor, we set all their properties at once through a list.
        List<DcMotorEx> motors = Arrays.asList(LeftFrontSwerveMotor, LeftBackSwerveMotor, RightBackSwerveMotor, RightFrontSwerveMotor);
        for(int i = 0; i < motors.size(); i++) {
            DcMotorEx motor = motors.get(i);
            if (i % 2 == 0) motor.setDirection(DcMotorSimple.Direction.REVERSE);
            else motor.setDirection(DcMotorSimple.Direction.FORWARD);
//            Anything in here will get repeated for each individual "motor" in the list
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

//        Initialize the other stuff here now
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        swerveDrive = new SwerveDriveProgramming(motors, imu);

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    @Override
    public void run() {
        super.run();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        swerveDrive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        telemetry.addLine("Swerve Test is Running");
        telemetry.update();

    }
}
