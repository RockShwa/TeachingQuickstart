
package org.firstinspires.ftc.teamcode.misc.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMapping {

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    // DRIVETRAIN
    // --------------
    public static double drive = 0.0;
    public static double strafe = 0.0;
    public static double turn = 0.0;

    // INTAKE (ACTIVE)
    // --------------
    public static Toggle extend;
    public static Toggle transfer;
    public static Toggle intakeOnToIntake;
    public static Toggle pivotToClear;
    public static Toggle clearIntake;
    public static Toggle pivotToClearSpec;

    // OUTTAKE
    // --------------
    public static Toggle highBasket;
    public static Toggle lowBasket;

    // SCORING
    // --------------
    public static Toggle scoreSpec;
    public static Toggle openClaw;
    public static Toggle hang;
    public static Toggle specMode;

    // LOCKED HEADING
    // -----------------
    // public static Toggle toggleLockedHeading;
    public static boolean lock90 = false;
    public static boolean lock180 = false;
    public static boolean lock270 = false;
    public static boolean lock360 = false;
    public static Toggle lockedMode;

    // OTHER
    // --------------
    public static Toggle botToBaseState;
    public static Toggle safeDeposit;

    public GamepadMapping(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        lockedMode = new Toggle(false);

        // INTAKE
        extend = new Toggle(false);
        intakeOnToIntake = new Toggle(false);
        pivotToClear = new Toggle(false);
        transfer = new Toggle(false);
        clearIntake = new Toggle(false);
        pivotToClearSpec = new Toggle(false);

        // OUTTAKE
        highBasket = new Toggle(false);
        lowBasket = new Toggle(false);
        hang = new Toggle(false);

        // spec
        openClaw = new Toggle(false);
        scoreSpec = new Toggle(false);


        // OTHER
        botToBaseState = new Toggle(false);
        specMode = new Toggle(false);
        safeDeposit = new Toggle(false);
    }

    public void joystickUpdate() {
        if(gamepad1.x){
            drive = gamepad1.left_stick_y*0.3;
            strafe = gamepad1.left_stick_x*0.3;
            turn = gamepad1.right_stick_x*0.3;
        }else {
            drive = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
        }
    }

    public void update() {
        joystickUpdate();

        // TODO: FIGURE THESE OUT
        //safeDeposit.update(gamepad2.dpad_right);
        hang.update(gamepad2.dpad_down);
        botToBaseState.update(gamepad2.left_stick_button);

        // intake
        activeIntakeUpdate();
        extend.update(gamepad1.right_bumper);

        // outtake
        lowBasket.update(gamepad2.left_trigger > 0.3);
        highBasket.update(gamepad2.left_bumper);

        // spec
        openClaw.update(gamepad2.right_trigger > 0.3);
        scoreSpec.update(gamepad2.right_bumper);
        specMode.update(gamepad2.dpad_right);

        // other
        // lockedMode.update(gamepad2.touchpad);
    }

    public void activeIntakeUpdate() {
        intakeOnToIntake.update(gamepad1.right_trigger > 0.5);
        pivotToClear.update(gamepad1.left_trigger > 0.5);
        transfer.update(gamepad2.dpad_up);
        pivotToClearSpec.update(gamepad1.left_bumper);
        clearIntake.update(gamepad1.x);
    }

    public void presModeUpdate() {
        extend.update(gamepad1.right_bumper);
        intakeOnToIntake.update(gamepad1.right_trigger > 0.5);

        transfer.update(gamepad1.dpad_up);

        highBasket.update(gamepad1.left_bumper);
        openClaw.update(gamepad1.left_trigger > 0.5);
    }

    public void resetIntakeControls() {
        extend.set(false);
        intakeOnToIntake.set(false);
        pivotToClear.set(false);
        transfer.set(false);
        clearIntake.set(false);
    }

    public void resetOuttakeControls() {
        highBasket.set(false);
        lowBasket.set(false);
        openClaw.set(false);
    }

    public void resetSpecControls() {
        openClaw.set(false);
        scoreSpec.set(false);
    }

    public void resetMultipleControls(Toggle... toggles) {
        for (Toggle toggle : toggles) {
            toggle.set(false);
        }
    }

    public void resetAllControls() {
        extend.set(false);
        intakeOnToIntake.set(false);
        pivotToClear.set(false);
        transfer.set(false);
        clearIntake.set(false);
        openClaw.set(false);
        scoreSpec.set(false);
        specMode.set(false);
        highBasket.set(false);
        lowBasket.set(false);
        pivotToClearSpec.set(false);
    }
}
