package org.firstinspires.ftc.teamcode.teaching.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOp {
    DcMotorEx leftMotor;

    public TeleOp(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(1000);
    }

    public void extend() {
        leftMotor.setPower(1);

    }
}
