package org.firstinspires.ftc.teamcode.input;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class GamepadTrigger {

    ArrayList<TriggerListener> triggerFullPressListenerArrayList = new ArrayList<>();
    ArrayList<TriggerListener> triggerReleaseListenerArrayList = new ArrayList<>();
    ArrayList<TriggerListener> triggerPressListenerArrayList = new ArrayList<>();

    double value;

    public interface TriggerListener {
        void execute(double value);
    }

    public void addFullPressListener(TriggerListener listener) {
        triggerFullPressListenerArrayList.add(listener);
    }

    public void removeFullPressListener(TriggerListener listener) {
        triggerFullPressListenerArrayList.remove(listener);
    }

    public void clearFullPressListeners() {
        triggerFullPressListenerArrayList.clear();
    }

    public void addPressListener(TriggerListener listener) {
        triggerPressListenerArrayList.add(listener);
    }

    public void removePressListener(TriggerListener listener) {
        triggerPressListenerArrayList.remove(listener);
    }

    public void clearPressListeners() {
        triggerPressListenerArrayList.clear();
    }

    public void addTriggerReleaseListener(TriggerListener listener) {
        triggerReleaseListenerArrayList.add(listener);
    }

    public void removeTriggerReleaseListener(TriggerListener listener) {
        triggerReleaseListenerArrayList.remove(listener);
    }

    public void clearTriggerReleaseListeners() {
        triggerReleaseListenerArrayList.clear();
    }

    void update(double newValue) {
        if(newValue != value) {
            if(newValue == 1) {
                onFullPress();
            } else if(newValue < 1 && value == 1) {
                onRelease();
            } else {
                onPress(newValue);
            }
            value = newValue;
        }
    }

    void onFullPress() {
        for(TriggerListener listener: triggerFullPressListenerArrayList) {
            listener.execute(1);
        }
    }

    void onPress(double value) {
        for(TriggerListener listener: triggerPressListenerArrayList) {
            listener.execute(value);
        }
    }

    void onRelease() {
        for(TriggerListener listener: triggerReleaseListenerArrayList) {
            listener.execute(0);
        }
    }

}