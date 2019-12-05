package frc.team5115.robot;

public class PID {

    double p, i, d = 0;
    double integral= 0;
    double previousError;
    boolean reset = false;

    Heartbeat timer;

    public PID(double p, double i, double d){
        timer = new Heartbeat();
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double calculate(double setpoint, double currentValue){
        timer.check();

        double currentError = currentValue - setpoint;
        integral += currentError * (timer.getDifference() / 1000);

        double derivative = (currentError - previousError) / (timer.getDifference() / 1000);
        previousError = currentError;

        double output = p * currentError + i * integral + d * derivative;
        if(output > 1 || output < -1){
            output = Math.signum(output);
        }
        return output;
    }

    public void reset(){
        previousError = 0;
        integral = 0;
        timer.start();
    }

}


