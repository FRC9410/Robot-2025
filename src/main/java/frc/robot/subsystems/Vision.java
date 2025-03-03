package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.Utils;

import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;

public class Vision extends SubsystemBase {
    private final BiConsumer<String, Object> updateData;
    private final NetworkTable leftTable;
    private final NetworkTable rightTable;
    final List<Integer> blueTagIds = Arrays.asList(12, 13, 16, 17, 18, 19, 20, 21, 22);
    final List<Integer> redTagIds = Arrays.asList(1, 2, 3, 6, 7, 8, 9, 10, 11);
    final List<Integer> tagIds;
    
    public Vision(BiConsumer<String, Object> updateData) {
        // Example camera initialization - you would add your actual cameras here
        // addCamera("front", "limelight-front", LimelightVersion.V4);
        // addCamera("back", "limelight-back", LimelightVersion.V3);

        this.updateData = updateData;
        
        leftTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_TABLE);
        rightTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_TABLE);

        tagIds = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redTagIds : blueTagIds;
    }

    @Override
    public void periodic() {

    }

    public NetworkTable getLeftTable() {
        return leftTable;
    }

    public NetworkTable getRightTable() {
        return rightTable;
    }

    public double getArea(NetworkTable table) {
        return table.getEntry("ta").getDouble(0.0);
    }

    public double getXOffset(NetworkTable table) {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getYOffset(NetworkTable table) {
        return table.getEntry("ty").getDouble(0.0);
    }
    public int getTagId(NetworkTable table) {
        return (int) table.getEntry("tid").getInteger(0);
    }

    public String getBestLimelight() {
        LimelightHelpers.PoseEstimate leftPerimeterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        LimelightHelpers.PoseEstimate rightPerimeterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
    
        String bestLimelight = "";
    
        if (leftPerimeterMeasurement != null
            && tagIds.contains(getTagId(leftTable))) {
          bestLimelight = "limelight-left";
        }
    
        if (rightPerimeterMeasurement != null
        && tagIds.contains(getTagId(rightTable))
        && ((leftPerimeterMeasurement != null  && rightPerimeterMeasurement.avgTagArea > leftPerimeterMeasurement.avgTagArea)
            || bestLimelight.isEmpty())) {
          bestLimelight = "limelight-right";
        }

        return bestLimelight;

    }
} 