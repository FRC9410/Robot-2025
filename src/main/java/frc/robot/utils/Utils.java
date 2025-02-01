package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utils {
    /**
     * Gets the current alliance color (Red or Blue)
     * @return Alliance enum representing the current alliance color. 
     *         Returns null if alliance color is not yet available.
     */
    public static Alliance getAllianceColor() {
        return DriverStation.getAlliance().orElse(null);
    }

    /**
     * Checks if a value is within a specified tolerance of a target
     * @param current The current value
     * @param target The target value
     * @param tolerance The acceptable tolerance (must be positive)
     * @return true if the current value is within the tolerance of the target
     */
    public static boolean isWithinTolerance(double current, double target, double tolerance) {
        return Math.abs(current - target) <= Math.abs(tolerance);
    }

    /**
     * Applies a curve to driver input to allow for finer control at low speeds
     * while still allowing for full speed. Preserves the sign of the input.
     * 
     * @param input The raw driver input (should be between -1.0 and 1.0)
     * @param exponent The power to raise the input to (2 for square, 3 for cube, etc.)
     * @return The curved value, maintaining the sign of the input
     */
    public static double curveInput(double input, double exponent) {
        double sign = Math.signum(input);
        return sign * Math.pow(Math.abs(input), exponent);
    }

    /**
     * Applies a squared curve to driver input to allow for finer control at low speeds
     * while still allowing for full speed. Preserves the sign of the input.
     * 
     * @param input The raw driver input (should be between -1.0 and 1.0)
     * @return The curved value, maintaining the sign of the input
     */
    public static double squareInput(double input) {
        return curveInput(input, 2.0);
    }

    /**
     * Applies a cubed curve to driver input to allow for finer control at low speeds
     * while still allowing for full speed. Preserves the sign of the input.
     * 
     * @param input The raw driver input (should be between -1.0 and 1.0)
     * @return The curved value, maintaining the sign of the input
     */
    public static double cubeInput(double input) {
        return curveInput(input, 3.0);
    }
} 