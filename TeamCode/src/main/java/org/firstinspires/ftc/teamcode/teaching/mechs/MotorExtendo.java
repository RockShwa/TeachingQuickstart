package org.firstinspires.ftc.teamcode.teaching.mechs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorExtendo {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    public MotorExtendo(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right");

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(1000); // this is some arbitrary value, you would have to test this
        rightMotor.setTargetPosition(1000);
    }

    public void extend() {
        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }
}
