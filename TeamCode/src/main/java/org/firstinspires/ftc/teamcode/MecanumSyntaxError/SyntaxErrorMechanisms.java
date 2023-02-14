package org.firstinspires.ftc.teamcode.MecanumSyntaxError;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Mecanum20D54D.PID;

public class SyntaxErrorMechanisms {
    final private DcMotor l;
    final private DcMotor r;
    final private CRServo il;
    final private CRServo ir;

    PID pid = new PID(0.008, 0, 0);

    public SyntaxErrorMechanisms(DcMotor lm, DcMotor rm, CRServo ils, CRServo ilr) {
        l = lm;
        r = rm;
        il = ils;
        ir = ilr;
    }
    public void runManual(double dr4bp, double ip) {
        l.setPower(-dr4bp - 0.11);
        r.setPower(dr4bp + 0.11);
        il.setPower(ip);
        ir.setPower(ip);
    }
    public void runPID(double target) {
        double command = pid.update(l.getCurrentPosition(), target) + 0.11;

        l.setPower(-command);
        r.setPower(command);
    }
    public double getPosition() {
        return r.getCurrentPosition();
    }
}
