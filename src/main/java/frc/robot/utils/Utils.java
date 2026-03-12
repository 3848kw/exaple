package frc.robot.utils;

public class Utils {
    /**
     * remaps a value within one range of numbers to increase and decrease within another range of numbers
     * 
     * @param value value to be remapped
     * @param from1 minimum number in first range
     * @param to1 manimum number in first range
     * @param from2 minimum number in second range
     * @param to2 maximum number in second range
     * @return
     */
    public static double remap(double value, double from1, double to1, double from2, double to2) {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
}
