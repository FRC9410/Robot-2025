package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final NetworkTable leftPerimeterTable;
    private final NetworkTable rightPerimeterTable;
    private final NetworkTable leftReefTable;
    private final NetworkTable rightReefTable;
    final List<Integer> reefTagIds = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    final List<Integer> bargeTagIds = Arrays.asList(4, 5, 14, 15);
    
    public Vision(BiConsumer<String, Object> updateData) {
        // Example camera initialization - you would add your actual cameras here
        // addCamera("front", "limelight-front", LimelightVersion.V4);
        // addCamera("back", "limelight-back", LimelightVersion.V3);

        this.updateData = updateData;
        
        leftPerimeterTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_PERIMETER_TABLE);
        rightPerimeterTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_PERIMETER_TABLE);
        leftReefTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_REEF_TABLE);
        rightReefTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_REEF_TABLE);
    }

    @Override
    public void periodic() {
        // updateData.accept("leftrfTA", getArea(getLeftReefTable()));
        // updateData.accept("rightrfT", getArea(getRightReefTable()));
        // updateData.accept("leftXOffset", getXOffset(getLeftReefTable()));
        // updateData.accept("rightXOffset", getXOffset(getRightReefTable()));

    }

    public NetworkTable getLeftReefTable() {
        return leftReefTable;
    }

    public NetworkTable getRightReefTable() {
        return rightReefTable;
    }

    public PoseEstimate getPose(boolean getBlueSide) {
        final double leftArea = getArea(leftPerimeterTable);
        final double rightArea = getArea(rightPerimeterTable);

        if (leftArea > rightArea) {
            return Utils.getAllianceColor() == "blue" || getBlueSide
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.LEFT_PERIMETER_TABLE)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.VisionConstants.RIGHT_PERIMETER_TABLE);
        } else {
            return Utils.getAllianceColor() == "blue" || getBlueSide
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.LEFT_PERIMETER_TABLE)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.VisionConstants.RIGHT_PERIMETER_TABLE);
        }
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

    public int getReefTag() {
        final double leftReefTagArea = getArea(leftReefTable);
        final double rightReefTagArea = getArea(rightReefTable);

        if (leftReefTagArea > 0.5 || rightReefTagArea > 0.5) {
            if (leftReefTagArea > rightReefTagArea) {
                return getTagId(leftReefTable);
            } else {
                return getTagId(rightReefTable);
            }
        } else {
            return 0;
        }
    }

    public String getBestLimelight() {
        LimelightHelpers.PoseEstimate leftPerimeterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-lftper");
        LimelightHelpers.PoseEstimate rightPerimeterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rghtper");
        LimelightHelpers.PoseEstimate leftReefMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-lftrf");
        LimelightHelpers.PoseEstimate rightReefMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rghtrf");
    
        String bestLimelight = "";
    
        if (leftPerimeterMeasurement != null
            && !reefTagIds.contains(getTagId(leftPerimeterTable))
            && !bargeTagIds.contains(getTagId(leftPerimeterTable))) {
          bestLimelight = "limelight-lftper";
        }
    
        if (rightPerimeterMeasurement != null
        && !reefTagIds.contains(getTagId(rightPerimeterTable))
        && !bargeTagIds.contains(getTagId(rightPerimeterTable))
        && ((leftPerimeterMeasurement != null  && rightPerimeterMeasurement.avgTagArea > leftPerimeterMeasurement.avgTagArea)
            || bestLimelight.isEmpty())) {
          bestLimelight = "limelight-rghtper";
        }
    
        if (leftReefMeasurement != null
        && !reefTagIds.contains(getTagId(leftReefTable))
        && ((LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight) != null && LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight).avgTagArea < leftReefMeasurement.avgTagArea)
            || bestLimelight.isEmpty())) {
          bestLimelight = "limelight-lftrf";
        }
    
        if (rightReefMeasurement != null
        && !reefTagIds.contains(getTagId(rightReefTable))
        && ((LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight) != null && LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight).avgTagArea < rightReefMeasurement.avgTagArea)
            || bestLimelight.isEmpty())) {
          bestLimelight = "limelight-rghtrf";
        }

        return bestLimelight;

    }
} 