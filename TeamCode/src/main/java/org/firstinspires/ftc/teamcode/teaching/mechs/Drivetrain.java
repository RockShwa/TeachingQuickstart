package org.firstinspires.ftc.teamcode.teaching.mechs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.misc.PIDFControllerEx;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class Drivetrain {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private IMU imu;

    private GamepadMapping controls;

    private double newX = 0;
    private double newY = 0;
    private double targetAngle = 0;

    // public while being tuned on dashboard
    public static double turnP = 0.013;
    public static double turnI = 0;
    public static double turnD = 0.0001;
    public static double turnF = 0;
    private PIDFControllerEx turnController = new PIDFControllerEx(turnP, turnI, turnD, turnF);

    private DriveMode driveMode;
    private boolean lockedHeadingMode = false;
    private double slowMultiplier = 1;

    public Drivetrain(HardwareMap hardwareMap, IMU imu, GamepadMapping controls){
        // motors for slingshot bot

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.imu = imu;

        this.controls = controls;

        driveMode = DriveMode.ROBO_CENTRIC;
    }

    public void update() {
        double strafe = controls.strafe;
        double drive = controls.drive;
        double turn = controls.turn;

        if (driveMode.equals(DriveMode.FIELD_CENTRIC)) {
            moveFieldCentric(strafe, drive, turn, getHeading());
        } else if (driveMode.equals(DriveMode.ROBO_CENTRIC)){
            moveRoboCentric(-strafe, drive, -turn);
        }
    }

    public void moveRoboCentric(double strafe, double drive, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        strafe = Math.tan(strafe);
        drive = Math.tan(drive);
        turn = Math.tan(turn);

        leftFront.setPower(((drive + strafe + turn) / denominator) * slowMultiplier);
        leftBack.setPower(((drive - strafe + turn) / denominator) * slowMultiplier);
        rightFront.setPower(((drive - strafe - turn) / denominator) * slowMultiplier);
        rightBack.setPower(((drive + strafe - turn) / denominator) * slowMultiplier);
    }

    public void moveFieldCentric(double inX, double inY, double turn, double currentAngle){
        currentAngle += 90;
        double radian = Math.toRadians(currentAngle);
        double cosTheta = Math.cos(radian);
        double sinTheta = Math.sin(radian);
        newX = (-inX * sinTheta) - (inY * cosTheta);
        newY = (-inX * cosTheta) + (inY * sinTheta);

        moveRoboCentric(newX,newY,-turn);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double angleWrap(double angle) {
        angle = Math.toRadians(angle);
        // Changes any angle between [-179,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI) {
            angle -= 2 * Math.PI * (angle > 0 ? 1 : -1); // if angle > 0 * 1, < 0 * -1
        }
        return Math.toDegrees(angle);
    }

    public void changePID(double inP, double inI, double inD, double inF){
        turnP = inP; turnI = inI; turnD = inD; turnF = inF;
        turnController.setPIDF(inP, inI, inD, inF);
    }

    public enum DriveMode {
        FIELD_CENTRIC,
        ROBO_CENTRIC
    }
}
