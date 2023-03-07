package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDStuff
{
    private double accumulatedError;
    private double lastTime;
    private double lastError;
    private double[] pidData = {0.1, 0.1, 0.1};
    private ElapsedTime timer = new ElapsedTime();

    public PIDStuff()
    {
        accumulatedError = 0.0;
        lastError = 0.0;
        lastTime = 0.0;
    }

    private double PIDControl(double reference, double currentState, double[] coefficients)
    {
        double error = reference - currentState;
        double time = timer.milliseconds();
        timer.reset();
        accumulatedError += (0.5 * (error + lastError)) * (time - lastTime);
        double derivative = 0;
        if (time - lastTime > 0)
        {
            derivative = (error - lastError) / (time - lastTime);
        }
        double output = coefficients[0] * error + coefficients[1] * accumulatedError + coefficients[2] * derivative;
        lastError = error;
        lastTime = time;
        return (0.1 * Math.signum(error) + 0.9 * Math.tanh(output));
    }

    private double PIDControl(double error, double[] coefficients)
    {
        double time = timer.milliseconds();
        timer.reset();
        accumulatedError += (0.5 * (error + lastError)) * (time - lastTime);
        double derivative = 0;
        if (time - lastTime > 0)
        {
            derivative = (error - lastError) / (time - lastTime);
        }
        double output = coefficients[0] * error + coefficients[1] * accumulatedError + coefficients[2] * derivative;
        lastError = error;
        lastTime = time;
        return (0.1 * Math.signum(error) + 0.9 * Math.tanh(output));
    }
    /*double fLPower = PIDControl(frontleft.getTargetPosition(), frontleft.getCurrentPosition(), pidData);
    double fRPower = PIDControl(frontright.getTargetPosition(), frontright.getCurrentPosition(), pidData);
    double bLPower = PIDControl(backleft.getTargetPosition(), backleft.getCurrentPosition(), pidData);
    double bRPower = PIDControl(backright.getTargetPosition(), backright.getCurrentPosition(), pidData);
    power(fLPower, fRPower, bLPower, bRPower);*/
}
