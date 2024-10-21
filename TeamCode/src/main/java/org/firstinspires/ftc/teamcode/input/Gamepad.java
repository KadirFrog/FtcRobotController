package org.firstinspires.ftc.teamcode.input;

public class Gamepad {

    public GamepadAnalogStick left_stick = new GamepadAnalogStick();
    public GamepadAnalogStick right_stick = new GamepadAnalogStick();

    public GamepadAnalogSticks both_sticks = new GamepadAnalogSticks();

    public GamepadButton left_stick_button = new GamepadButton();
    public GamepadButton right_stick_button = new GamepadButton();

    public GamepadTrigger left_trigger = new GamepadTrigger();
    public GamepadTrigger right_trigger = new GamepadTrigger();

    public GamepadButton left_bumper = new GamepadButton();
    public GamepadButton right_bumper = new GamepadButton();

    public GamepadButton a = new GamepadButton();
    public GamepadButton b = new GamepadButton();
    public GamepadButton x = new GamepadButton();
    public GamepadButton y = new GamepadButton();

    public GamepadButton dpad_up = new GamepadButton();
    public GamepadButton dpad_right = new GamepadButton();
    public GamepadButton dpad_down = new GamepadButton();
    public GamepadButton dpad_left = new GamepadButton();

    void update(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        left_stick.update(gamepad.left_stick_x, gamepad.left_stick_y);
        right_stick.update(gamepad.right_stick_x, gamepad.right_stick_y);

        both_sticks.update(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, gamepad.right_stick_y);

        left_stick_button.update(gamepad.left_stick_button);
        right_stick_button.update(gamepad.right_stick_button);

        left_trigger.update(gamepad.left_trigger);
        right_trigger.update(gamepad.right_trigger);

        left_bumper.update(gamepad.left_bumper);
        right_bumper.update(gamepad.right_bumper);

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        dpad_up.update(gamepad.dpad_up);
        dpad_right.update(gamepad.dpad_right);
        dpad_down.update(gamepad.dpad_down);
        dpad_left.update(gamepad.dpad_left);
    }

}