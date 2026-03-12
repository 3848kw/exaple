package frc.robot.utils;

public class PIDUtils {
    /**
     * Returns the nearest congruent (equilivant) rotation of the desired value to the current value.
     * Christian was here.
     * @param desired - the desired rotation, in degrees
     * @param current - the current rotation, in degrees
     * @return the nearest congruent rotation to target
     */
    public static double nearestCongruentRotationDegrees(double desired, double current) {
        desired = desired % 360;
        if (desired < 0) desired += 360;
        double option_1 = desired + (360.0 * Math.floor(current / 360.0));
        double option_2 = option_1 + 360.0;
        double option_3 = option_1 - 360.0;
        double error_1 = Math.abs(current - option_1);
        double error_2 = Math.abs(current - option_2);
        double error_3 = Math.abs(current - option_3);
        if (error_1 < error_2 && error_1 < error_3) return option_1;
        return error_2 < error_3 ? option_2 : option_3;
    }
}
