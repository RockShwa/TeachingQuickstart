
package org.firstinspires.ftc.teamcode.misc;

public class PIDFControllerEx {
    //    public PIDFControllerEx(double kp, double ki, double kd, double kf) {
//        super(kp, ki, kd, kf);
//    }
//
//    @Override
//    public double calculate(double error) {
//        // we did this to add the ability to control the state of our error so heading lock would work
//        super.setSetPoint(0);
//        return super.calculate(-error);
//    }
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double setPoint;
    private double measuredValue;
    private double minIntegral;
    private double maxIntegral;
    private double errorVal_p;
    private double errorVal_v;
    private double totalError;
    private double prevErrorVal;
    private double errorTolerance_p;
    private double errorTolerance_v;
    private double lastTimeStamp;
    private double period;

    public PIDFControllerEx(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0.0, 0.0);
    }

    public PIDFControllerEx(double kp, double ki, double kd, double kf, double sp, double pv) {
        this.errorTolerance_p = 0.05;
        this.errorTolerance_v = Double.POSITIVE_INFINITY;
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.kF = kf;
        this.setPoint = sp;
        this.measuredValue = pv;
        this.minIntegral = -1.0;
        this.maxIntegral = 1.0;
        this.lastTimeStamp = 0.0;
        this.period = 0.0;
        this.errorVal_p = this.setPoint - this.measuredValue;
        this.reset();
    }

    public void reset() {
        this.totalError = 0.0;
        this.prevErrorVal = 0.0;
        this.lastTimeStamp = 0.0;
    }

    public void setTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        this.errorTolerance_p = positionTolerance;
        this.errorTolerance_v = velocityTolerance;
    }

    public double getSetPoint() {
        return this.setPoint;
    }

    public void setSetPoint(double sp) {
        this.setPoint = sp;
        this.errorVal_p = this.setPoint - this.measuredValue;
        this.errorVal_v = (this.errorVal_p - this.prevErrorVal) / this.period;
    }

    public boolean atSetPoint() {
        return Math.abs(this.errorVal_p) < this.errorTolerance_p && Math.abs(this.errorVal_v) < this.errorTolerance_v;
    }

    public double[] getCoefficients() {
        return new double[]{this.kP, this.kI, this.kD, this.kF};
    }

    public double getPositionError() {
        return this.errorVal_p;
    }

    public double[] getTolerance() {
        return new double[]{this.errorTolerance_p, this.errorTolerance_v};
    }

    public double getVelocityError() {
        return this.errorVal_v;
    }

    public double calculate() {
        return this.calculate(this.measuredValue);
    }

    public double calculate(double pv, double sp) {
        this.setSetPoint(sp);
        return this.calculate(pv);
    }

    public double calculate(double error, boolean addF) {
        // we did this to add the ability to control the state of our error so heading lock would work
        this.setSetPoint(-error);
        if (addF) {
            return this.calculate(-0) + this.kF;
        }
        return this.calculate(-0);
    }

    public double calculate(double pv) {
        this.prevErrorVal = this.errorVal_p;
        double currentTimeStamp = (double)System.nanoTime() / 1.0E9;
        if (this.lastTimeStamp == 0.0) {
            this.lastTimeStamp = currentTimeStamp;
        }

        this.period = currentTimeStamp - this.lastTimeStamp;
        this.lastTimeStamp = currentTimeStamp;
        if (this.measuredValue == pv) {
            this.errorVal_p = this.setPoint - this.measuredValue;
        } else {
            this.errorVal_p = this.setPoint - pv;
            this.measuredValue = pv;
        }

        if (Math.abs(this.period) > 1.0E-6) {
            this.errorVal_v = (this.errorVal_p - this.prevErrorVal) / this.period;
        } else {
            this.errorVal_v = 0.0;
        }

        this.totalError += this.period * (this.setPoint - this.measuredValue);
        this.totalError = this.totalError < this.minIntegral ? this.minIntegral : Math.min(this.maxIntegral, this.totalError);
        return this.kP * this.errorVal_p + this.kI * this.totalError + this.kD * this.errorVal_v + this.kF * this.setPoint;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.kF = kf;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        this.minIntegral = integralMin;
        this.maxIntegral = integralMax;
    }

    public void clearTotalError() {
        this.totalError = 0.0;
    }

    public void setP(double kp) {
        this.kP = kp;
    }

    public void setI(double ki) {
        this.kI = ki;
    }

    public void setD(double kd) {
        this.kD = kd;
    }

    public void setF(double kf) {
        this.kF = kf;
    }

    public double getP() {
        return this.kP;
    }

    public double getI() {
        return this.kI;
    }

    public double getD() {
        return this.kD;
    }

    public double getF() {
        return this.kF;
    }

    public double getPeriod() {
        return this.period;
    }
}
