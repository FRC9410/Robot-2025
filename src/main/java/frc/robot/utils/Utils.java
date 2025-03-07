package frc.robot.utils;

import java.awt.Point;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ReefConstants;
import frc.robot.Constants.WaypointConstants;

public class Utils {
    /**
     * Gets the current alliance color (Red or Blue)
     * @return Alliance enum representing the current alliance color. 
     *         Returns null if alliance color is not yet available.
     */
    public static String getAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red ? "red" : "blue";
        }
        return "";
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
        if (input < 0) {
            return -curveInput(-input, 2.0);
        }
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

    public static void logMap(Map<String, Object> map, String tableName) {
        final NetworkTableInstance inst = NetworkTableInstance.getDefault();
        final NetworkTable table = inst.getTable(tableName);
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            if (entry.getValue() == null || entry.getKey() == null) {
                System.out.println(entry.getKey());
                continue;
            }
            if (entry.getValue() instanceof Double) {
                table.getEntry(entry.getKey()).setDouble((Double) entry.getValue());
            } else if (entry.getValue() instanceof Integer) {
                table.getEntry(entry.getKey()).setNumber((Integer) entry.getValue());
            } else if (entry.getValue() instanceof Boolean) {
                table.getEntry(entry.getKey()).setBoolean((Boolean) entry.getValue());
            } else {
            table.getEntry(entry.getKey()).setString(entry.getValue().toString());
            }
        }
    }
    
    // Define hexagon vertices (assuming known, in counterclockwise order)
    private static final List<Point2D.Double> BLUE_HEXAGON_VERTICES = Arrays.asList(
        ReefConstants.BLUE_BACK_LEFT,
        ReefConstants.BLUE_FRONT_LEFT,
        ReefConstants.BLUE_FRONT_RIGHT,
        ReefConstants.BLUE_BACK_RIGHT,
        ReefConstants.BLUE_LEFT,
        ReefConstants.BLUE_RIGHT
    );

    private static final List<Point2D.Double> RED_HEXAGON_VERTICES = Arrays.asList(
        ReefConstants.RED_BACK_LEFT,
        ReefConstants.RED_FRONT_LEFT,
        ReefConstants.RED_FRONT_RIGHT,
        ReefConstants.RED_BACK_RIGHT,
        ReefConstants.RED_LEFT,
        ReefConstants.RED_RIGHT
    );

    // Define predefined safe waypoints around the hexagon
    private static final List<Point2D.Double> BLUE_SAFE_WAYPOINTS = Arrays.asList(
        new Point2D.Double(ReefConstants.BLUE_BACK_LEFT.getX(), ReefConstants.BLUE_BACK_LEFT.getY()),
        new Point2D.Double(ReefConstants.BLUE_FRONT_LEFT.getX(), ReefConstants.BLUE_FRONT_LEFT.getY()),
        new Point2D.Double(ReefConstants.BLUE_FRONT_RIGHT.getX(), ReefConstants.BLUE_FRONT_RIGHT.getY()),
        new Point2D.Double(ReefConstants.BLUE_BACK_RIGHT.getX(), ReefConstants.BLUE_BACK_RIGHT.getY()),
        new Point2D.Double(ReefConstants.BLUE_LEFT.getX(), ReefConstants.BLUE_LEFT.getY()),
        new Point2D.Double(ReefConstants.BLUE_RIGHT.getX(), ReefConstants.BLUE_RIGHT.getY())
    );

    private static final List<Point2D.Double> RED_SAFE_WAYPOINTS = Arrays.asList(
        new Point2D.Double(ReefConstants.RED_BACK_LEFT.getX(), ReefConstants.RED_BACK_LEFT.getY()),
        new Point2D.Double(ReefConstants.RED_FRONT_LEFT.getX(), ReefConstants.RED_FRONT_LEFT.getY()),
        new Point2D.Double(ReefConstants.RED_FRONT_RIGHT.getX(), ReefConstants.RED_FRONT_RIGHT.getY()),
        new Point2D.Double(ReefConstants.RED_BACK_RIGHT.getX(), ReefConstants.RED_BACK_RIGHT.getY()),
        new Point2D.Double(ReefConstants.RED_LEFT.getX(), ReefConstants.RED_LEFT.getY()),
        new Point2D.Double(ReefConstants.RED_RIGHT.getX(), ReefConstants.RED_RIGHT.getY())
    );

    // Define robot width
    private static final double ROBOT_WIDTH = 1.5; // Adjust as needed

    /**
     * Checks if the given line (robot's path) or its offset lines intersect the hexagon.
     */
    public static boolean pathIntersectsHexagon(Point2D.Double start, Point2D.Double target) {
        final List<Point2D.Double> HEXAGON_VERTICES = getAllianceColor().equals("red") ? RED_HEXAGON_VERTICES : BLUE_HEXAGON_VERTICES;
        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            Point2D.Double p1 = HEXAGON_VERTICES.get(i);
            Point2D.Double p2 = HEXAGON_VERTICES.get((i + 1) % HEXAGON_VERTICES.size());

            // Check if the main path intersects
            if (Line2D.linesIntersect(start.x, start.y, target.x, target.y, p1.x, p1.y, p2.x, p2.y)) {
                return true;
            }

            // Check if the offset paths intersect
            Point2D.Double leftOffsetStart = offsetPoint(start, target, -ROBOT_WIDTH / 2);
            Point2D.Double leftOffsetEnd = offsetPoint(target, start, -ROBOT_WIDTH / 2);
            Point2D.Double rightOffsetStart = offsetPoint(start, target, ROBOT_WIDTH / 2);
            Point2D.Double rightOffsetEnd = offsetPoint(target, start, ROBOT_WIDTH / 2);

            if (Line2D.linesIntersect(leftOffsetStart.x, leftOffsetStart.y, leftOffsetEnd.x, leftOffsetEnd.y, p1.x, p1.y, p2.x, p2.y)
             || Line2D.linesIntersect(rightOffsetStart.x, rightOffsetStart.y, rightOffsetEnd.x, rightOffsetEnd.y, p1.x, p1.y, p2.x, p2.y)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Offsets a point perpendicular to the line formed by (start, target).
     */
    private static Point2D.Double offsetPoint(Point2D.Double start, Point2D.Double target, double offset) {
        double dx = target.x - start.x;
        double dy = target.y - start.y;
        double length = Math.sqrt(dx * dx + dy * dy);

        // Normalize perpendicular vector
        double perpX = -dy / length;
        double perpY = dx / length;

        // Apply perpendicular offset
        return new Point2D.Double(start.x + perpX * offset, start.y + perpY * offset);
    }

    /**
     * Finds the closest safe waypoint to the robot that does not cross the hexagon.
     */
    public static Pose2d findSafeWaypoint(Pose2d startPose, Pose2d targetPose) {
        final Point2D.Double start = new Point2D.Double(startPose.getX(), startPose.getY());
        final Point2D.Double target = new Point2D.Double(targetPose.getX(), targetPose.getY());
        final List<Point2D.Double> SAFE_WAYPOINTS = getAllianceColor().equals("red") ? RED_SAFE_WAYPOINTS : BLUE_SAFE_WAYPOINTS;
        Point2D.Double bestWaypoint = null;
        double minDistance = Double.MAX_VALUE;

        for (Point2D.Double waypoint : SAFE_WAYPOINTS) {
            if (!pathIntersectsHexagon(start, waypoint) && !pathIntersectsHexagon(waypoint, target)) {
                double distance = start.distance(waypoint);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestWaypoint = waypoint;
                }
            }
        }
        return new Pose2d(bestWaypoint.x, bestWaypoint.y, targetPose.getRotation());
    }
} 