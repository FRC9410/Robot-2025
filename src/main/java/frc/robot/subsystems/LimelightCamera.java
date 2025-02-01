package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

public class LimelightCamera {
    private final NetworkTable table;
    private final LimelightVersion version;
    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0;

    public enum LimelightVersion {
        V3, V4
    }

    public LimelightCamera(String tableName, LimelightVersion version) {
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
        this.version = version;
    }

    /**
     * @return true if the camera has any valid targets
     */
    public boolean hasValidTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * @return Horizontal offset from crosshair to target (-27 to 27 degrees)
     */
    public double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
     */
    public double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /**
     * Gets the robot's pose from the camera
     * @return The estimated pose of the robot
     */
    public Optional<Pose2d> getPose() {
        if (!hasValidTarget()) {
            return Optional.empty();
        }
        return Optional.of(lastPose);
    }

    /**
     * Updates the stored pose from the camera
     */
    public void updatePose() {
        if (!hasValidTarget()) {
            return;
        }

        double[] botpose = version == LimelightVersion.V4 ? 
            table.getEntry("botpose").getDoubleArray(new double[6]) :
            table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        if (botpose.length >= 6) {
            lastPose = new Pose2d(
                new Translation2d(botpose[0], botpose[1]),
                Rotation2d.fromDegrees(botpose[5])
            );
            lastTimestamp = table.getEntry("cl").getDouble(0.0);
        }
    }

    /**
     * @return The timestamp of the last pose update
     */
    public double getLastTimestamp() {
        return lastTimestamp;
    }

    /**
     * Sets the LED mode of the camera
     * @param mode 0 = pipeline default, 1 = force off, 2 = force blink, 3 = force on
     */
    public void setLEDMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the camera mode
     * @param mode 0 = Vision processor, 1 = Driver Camera
     */
    public void setCameraMode(int mode) {
        table.getEntry("camMode").setNumber(mode);
    }

    /**
     * Sets the current pipeline number
     * @param pipeline Pipeline number (0-9)
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Gets the latency of the camera pipeline
     * @return The latency in milliseconds
     */
    public double getLatency() {
        return table.getEntry("cl").getDouble(0.0) + 
               table.getEntry("tl").getDouble(0.0);
    }
} 