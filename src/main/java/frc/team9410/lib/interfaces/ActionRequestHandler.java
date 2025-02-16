package frc.team9410.lib.interfaces;

import java.util.Map;

import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.ActionController.Action;

public interface ActionRequestHandler {
    boolean matches(Map<String, Object> state, Action action);
    void execute(Map<String, Object> state, ActionController controller);
}