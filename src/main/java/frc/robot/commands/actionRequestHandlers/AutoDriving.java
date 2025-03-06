package frc.robot.commands.actionRequestHandlers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.MapConstants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.ActionController.Action;
import frc.team9410.lib.interfaces.ActionRequestHandler;

public class AutoDriving implements ActionRequestHandler {
    public boolean matches(Map<String, Object> state, Action action) {
        if (state.get(MapConstants.POSE) != null && state.get(MapConstants.HAS_PIECE) != null){
            return ((Boolean) state.get(MapConstants.HAS_PIECE));
        }
        return false;
    }
    
    public void execute(Map<String, Object> state, ActionController controller) {
        final Pose2d currentPose = (Pose2d) state.get(MapConstants.POSE);
        final Pose2d targetPose = (Pose2d) state.get(MapConstants.TARGET_POSE);


        controller.setCommandData(Map.of(
            MapConstants.TARGET_POSE, targetPose
        ));
    }
}