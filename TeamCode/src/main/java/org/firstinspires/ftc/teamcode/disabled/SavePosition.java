package org.firstinspires.ftc.teamcode.disabled;

import com.pedropathing.geometry.Pose;

public  class SavePosition  {
    static Pose savedPosition = new Pose(0,0,0);
    public static void saveCurrentPosition(Pose currentPose) {
        savedPosition = currentPose.copy();
    }
    public static Pose getSavedPosition() {
        return savedPosition;
    }

}
