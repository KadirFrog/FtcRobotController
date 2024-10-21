package org.firstinspires.ftc.teamcode.input;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class GamepadAnalogStick {

    ArrayList<UpdateListener> updateListenerArrayList = new ArrayList<>();

    public interface UpdateListener {
        void execute(double x, double y);
    }

    public void addUpdateListener(UpdateListener listener) {
        updateListenerArrayList.add(listener);
    }

    public void removeUpdateListener(UpdateListener listener) {
        updateListenerArrayList.remove(listener);
    }

    public void clearUpdateListenerList() {
        updateListenerArrayList.clear();
    }

    void update(double x, double y) {
        for (UpdateListener listener : updateListenerArrayList) {
            listener.execute(x, -y);
        }
    }

}