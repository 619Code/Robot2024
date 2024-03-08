package frc.robot.helpers;

import com.kauailabs.navx.frc.AHRS;

public class AutoSelector {
    private static AHRS gyro;
    public static void setGyro(AHRS g) {
        gyro = g;
    }

    public static boolean isFacingSide() {
        if (gyro != null) {
            double angle = gyro.getAngle();
            if (Math.abs(angle) > 15) {
                return true;
            } else return false;
        } else {
            System.out.println("Missing Gyro. :(");
            return false;
        }
    }

    public static boolean isfacingLeft() {
        if (gyro != null) {
            double angle = gyro.getAngle();
            if (angle < 0) {                            // SUBJECT TO CHANGE, INVERT IF NECESSARY!
                return true;
            } else return false;
        } else {
            System.out.println("Missing Gyro. :(");
            return false;
        }
    }

}