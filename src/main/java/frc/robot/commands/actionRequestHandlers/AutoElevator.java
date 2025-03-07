package frc.robot.commands.actionRequestHandlers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.MapConstants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.ActionController.Action;
import frc.team9410.lib.interfaces.ActionRequestHandler;
import frc.robot.utils.FieldLocations;

public class AutoElevator implements ActionRequestHandler {
    public boolean matches(Map<String, Object> state, Action action) {
        if (state.get(MapConstants.POSE) != null
        && state.get(MapConstants.HAS_PIECE) != null
        && state.get(MapConstants.TARGET_POSE) != null
        && state.get(MapConstants.ELEVATOR_POSITION) != null) {
            final Pose2d currentPose = (Pose2d) state.get(MapConstants.POSE);
            final Pose2d targetPose = (Pose2d) state.get(MapConstants.TARGET_POSE);
            final boolean isWithinTolerance = Math.abs(currentPose.getTranslation().getX() - targetPose.getTranslation().getX()) < AutoConstants.TRANSLATION_TOLERANCE &&
                Math.abs(currentPose.getTranslation().getY() - targetPose.getTranslation().getY()) < AutoConstants.TRANSLATION_TOLERANCE
                && Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()) < AutoConstants.ROTATION_TOLERANCE;
            return ((Boolean) state.get(MapConstants.HAS_PIECE)) && isWithinTolerance;
        }
        return false;
    }
    
    public void execute(Map<String, Object> state, ActionController controller) {


        controller.setCommandData(Map.of(
            MapConstants.ELEVATOR_POSITION, state.get(MapConstants.ELEVATOR_POSITION)
        ));
    }
}