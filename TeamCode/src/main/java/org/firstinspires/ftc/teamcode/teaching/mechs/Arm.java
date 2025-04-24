package org.firstinspires.ftc.teamcode.teaching.mechs;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Arm {
    private ServoImplEx claw;
    private ServoImplEx arm;

    public Arm(ServoImplEx claw, ServoImplEx arm) {
        this.claw = claw;
        this.arm = arm;
    }

    public void openClaw() {
        claw.setPosition(1);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }

    public void armToScore() {
        arm.setPosition(1);
    }

    public void lowerArm() {
        arm.setPosition(0);
    }
}
