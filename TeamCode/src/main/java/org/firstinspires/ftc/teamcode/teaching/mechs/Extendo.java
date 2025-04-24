package org.firstinspires.ftc.teamcode.teaching.mechs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Extendo {
    private ServoImplEx leftExtendo;
    private ServoImplEx rightExtendo;

    private DcMotorEx intake;
    private ServoImplEx pivot;

    public Extendo(ServoImplEx leftExtendo, ServoImplEx rightExtendo, DcMotorEx intake, ServoImplEx pivot) {
        this.leftExtendo = leftExtendo;
        this.rightExtendo = rightExtendo;
        this.intake = intake;
        this.pivot = pivot;
    }

    public void extend() {
        leftExtendo.setPosition(1);
        rightExtendo.setPosition(1);
    }

    public void retract() {
        leftExtendo.setPosition(0);
        rightExtendo.setPosition(0);
    }

    public void pivotDown() {
        pivot.setPosition(0.5);
    }

    public void pivotUp() {
        pivot.setPosition(0);
    }

    public void intakeOn() {
        intake.setPower(1);
    }

    public void clearOn() {
        intake.setPower(-1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }
}
