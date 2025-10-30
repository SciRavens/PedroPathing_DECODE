package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private final Servo servoTurret;
    private double target = 0.5;

    public Turret(Servo servoTurret) {
        this.servoTurret = servoTurret;
        this.target = servoTurret.getPosition();
    }

    public double getPosition() {
        return servoTurret.getPosition();
    }

    public void setTarget(double target) {
        this.target = clamp(target, 0.0, 1.0);
    }

    public void apply() {
        servoTurret.setPosition(target);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}

