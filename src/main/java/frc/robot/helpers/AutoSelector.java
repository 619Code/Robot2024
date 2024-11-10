package frc.robot.helpers;

import com.kauailabs.navx.frc.AHRS;

public class AutoSelector {
    private static AHRS gyro;
    public static void setGyro(AHRS g) {
        gyro = g;
    }

    public static boolean isFacingSide() {
        if (gyro != null) {
            double angle = gyro.getAngle() - 180;       // Inverted since we will be facing backwards.
            if (Math.abs(angle) > 15) {
                return true;
            } else return false;
        } else {
            return false;
        }
    }

    public static boolean isfacingLeft() {
        if (gyro != null) {
            double angle = gyro.getAngle() - 180;       // Inverted since we will be facing backwards.
            if (angle < 0) {                            // SUBJECT TO CHANGE, INVERT IF NECESSARY!
                return true;
            } else return false;

        } else {
            return false;
        }
    }

    public static AutoLocation getLocation() {
        if (gyro != null) {
            if (isFacingSide()) {
                if (isfacingLeft()) {
                    return AutoLocation.LEFT;
                } else return AutoLocation.RIGHT;
            } else return AutoLocation.CENTER;

        } else {
            return AutoLocation.NONE;
        }
    }

}