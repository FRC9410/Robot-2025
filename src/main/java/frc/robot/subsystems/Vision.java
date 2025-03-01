package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.Utils;

import java.util.List;
import java.util.function.BiConsumer;

public class Vision extends SubsystemBase {
    private final BiConsumer<String, Object> updateData;
    private final NetworkTable leftPerimeterTable;
    private final NetworkTable rightPerimeterTable;
    private final NetworkTable leftReefTable;
    private final NetworkTable rightReefTable;
    private final List<String> LocalizationTables;
    private final List<String> ServoTables;
    
    public Vision(BiConsumer<String, Object> updateData) {
        // Example camera initialization - you would add your actual cameras here
        // addCamera("front", "limelight-front", LimelightVersion.V4);
        // addCamera("back", "limelight-back", LimelightVersion.V3);

        this.updateData = updateData;
        
        leftPerimeterTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_PERIMETER_TABLE);
        rightPerimeterTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_PERIMETER_TABLE);
        leftReefTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_REEF_TABLE);
        rightReefTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_REEF_TABLE);

        LocalizationTables = List.of(
            Constants.VisionConstants.LEFT_PERIMETER_TABLE,
            Constants.VisionConstants.RIGHT_PERIMETER_TABLE
        );

        ServoTables = List.of(
            Constants.VisionConstants.LEFT_REEF_TABLE,
            Constants.VisionConstants.RIGHT_REEF_TABLE
        );
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
} 