package org.firstinspires.ftc.teamcode.input;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class GamepadButton {

    ArrayList<ButtonListener> buttonPressListenerArrayList = new ArrayList<>();
    ArrayList<ButtonListener> buttonReleaseListenerArrayList = new ArrayList<>();

    boolean state = false;

    public interface ButtonListener {
        void execute();
    }

    public void addButtonPressListener(ButtonListener listener) {
        buttonPressListenerArrayList.add(listener);
    }

    public void removeButtonPressListener(ButtonListener listener) {
        buttonPressListenerArrayList.remove(listener);
    }

    public void clearButtonPressListeners() {
        buttonPressListenerArrayList.clear();
    }

    public void addButtonReleaseListener(ButtonListener listener) {
        buttonReleaseListenerArrayList.add(listener);
    }

    public void removeButtonReleaseListener(ButtonListener listener) {
        buttonReleaseListenerArrayList.remove(listener);
    }

    public void clearButtonReleaseListeners() {
        buttonReleaseListenerArrayList.clear();
    }

    void update(boolean newState) {
        if(newState != state) {
            if(newState) {
                onPress();
            } else {
                onRelease();
            }
            state = newState;
        }
    }

    void onPress() {
        for(ButtonListener listener: buttonPressListenerArrayList) {
            listener.execute();
        }
    }

    void onRelease() {
        for(ButtonListener listener: buttonReleaseListenerArrayList) {
            listener.execute();
        }
    }

}
