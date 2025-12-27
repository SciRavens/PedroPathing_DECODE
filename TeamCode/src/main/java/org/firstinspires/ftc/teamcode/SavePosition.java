package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public  class SavePosition  {
    static Pose savedPosition = new Pose(0,0,0);
    public static void saveCurrentPosition(Pose currentPose) {
        savedPosition = currentPose;
    }
    public static Pose getSavedPosition() {
        return savedPosition;
    }

}
