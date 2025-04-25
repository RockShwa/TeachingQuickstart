package org.firstinspires.ftc.teamcode.teaching.mechs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Arm {
    private Servo claw;
    private Servo wrist;
    private Servo arm;

    public Arm(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm = hardwareMap.get(Servo.class, "arm");
    }

    public void openClaw() {
        claw.setPosition(1);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }


}
