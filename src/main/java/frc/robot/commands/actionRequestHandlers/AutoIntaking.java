package frc.robot.commands.actionRequestHandlers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.MapConstants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.ActionController.Action;
import frc.team9410.lib.interfaces.ActionRequestHandler;
import frc.robot.utils.FieldLocations;

public class AutoIntaking implements ActionRequestHandler {
    public boolean matches(Map<String, Object> state, Action action) {
        if (state.get(MapConstants.POSE) != null && state.get(MapConstants.HAS_PIECE) != null){
            return !((Boolean) state.get(MapConstants.HAS_PIECE));
        }
        return false;
    }
    
    public void execute(Map<String, Object> state, ActionController controller) {
        final Pose2d currentPose = (Pose2d) state.get(MapConstants.POSE);
        final double rotation;
        final double x;
        final double y;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            if (currentPose.getTranslation().getY() < FieldConstants.Y_MAX / 2) {
                rotation = 120.0;
                x = FieldConstants.X_MAX - 1.3;
                y = FieldConstants.Y_MIN + 0.9;
            } else {
                rotation = -120.0;
                x = FieldConstants.X_MAX - 1.3;
                y = FieldConstants.Y_MAX - 0.9;
            }
        } else {
            if (currentPose.getTranslation().getY() < FieldConstants.Y_MAX / 2) {
                rotation = 120.0;
                x = FieldConstants.X_MIN + 1.3;
                y = FieldConstants.Y_MIN + 0.9;
            } else {
                rotation = -120.0;
                x = FieldConstants.X_MIN + 1.3;
                y = FieldConstants.Y_MAX - 0.9;
            }
        }

        final double hopperVoltage = FieldLocations.isNearHp(currentPose) ? HopperConstants.START_VOLTAGE : HopperConstants.STOP_VOLTAGE;


        controller.setCommandData(Map.of(
            MapConstants.HOPPER_VOLTAGE, hopperVoltage,
            MapConstants.TARGET_POSE, new Pose2d(x, y, Rotation2d.fromDegrees(rotation))
        ));
    }
}