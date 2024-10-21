package org.firstinspires.ftc.teamcode.input;

public class InputManager {

    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();

    public void update(com.qualcomm.robotcore.hardware.Gamepad gamepad_1, com.qualcomm.robotcore.hardware.Gamepad gamepad_2) {
        gamepad1.update(gamepad_1);
        gamepad2.update(gamepad_2);
    }

}