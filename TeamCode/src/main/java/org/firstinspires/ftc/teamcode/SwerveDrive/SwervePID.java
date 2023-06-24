package org.firstinspires.ftc.teamcode.SwerveDrive;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwervePID {
    double integralSum = 0;
    private final double kp;
    private final double ki;
    private final double kd;
    private double lastError=0;
    ElapsedTime timer = new ElapsedTime();

    public SwervePID (double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double PIDControl(double state, double target) {
        double error = target - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        double output = (error * kp) + (derivative * kd) + (integralSum * ki);
        return output;
    }
}
