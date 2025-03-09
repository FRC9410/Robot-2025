package frc.robot.commands.actionRequestHandlers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.ScoringConstants;
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
        final Pose2d targetPose;

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            if (currentPose.getTranslation().getY() < FieldConstants.Y_MAX / 2) {
                targetPose = ScoringConstants.RED_HP_LEFT;
            } else {
                targetPose = ScoringConstants.RED_HP_RIGHT;
            }
        } else {
            if (currentPose.getTranslation().getY() < FieldConstants.Y_MAX / 2) {
                targetPose = ScoringConstants.BLUE_HP_RIGHT;
            } else {
                targetPose = ScoringConstants.BLUE_HP_LEFT;
            }
        }

        final double hopperVoltage = FieldLocations.isNearHp(currentPose) ? HopperConstants.START_VOLTAGE : HopperConstants.STOP_VOLTAGE;


        controller.setCommandData(Map.of(
            MapConstants.HOPPER_VOLTAGE, hopperVoltage,
            MapConstants.TARGET_POSE, targetPose
        ));
    }
}