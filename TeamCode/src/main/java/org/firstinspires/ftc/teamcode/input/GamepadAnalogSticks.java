package org.firstinspires.ftc.teamcode.input;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class GamepadAnalogSticks {

    ArrayList<UpdateListener> updateListenerArrayList = new ArrayList<>();

    public interface UpdateListener {
        void execute(double left_x, double left_y, double right_x, double right_y);
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

    void update(double left_x, double left_y, double right_x, double right_y) {
        for (UpdateListener listener : updateListenerArrayList) {
            listener.execute(left_x, -left_y, right_x, -right_y);
        }
    }

}