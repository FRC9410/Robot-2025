package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import frc.robot.subsystems.LimelightCamera.LimelightVersion;

public class VisionSubsystem extends SubsystemBase {
    private final Map<String, LimelightCamera> cameras = new HashMap<>();
    
    public VisionSubsystem() {
        // Example camera initialization - you would add your actual cameras here
        // addCamera("front", "limelight-front", LimelightVersion.V4);
        // addCamera("back", "limelight-back", LimelightVersion.V3);
    }

    /**
     * Adds a new camera to the vision subsystem
     * @param name Friendly name for the camera
     * @param networkTableName The network table name for the camera
     * @param version The Limelight version (V3 or V4)
     */
    public void addCamera(String name, String networkTableName, LimelightVersion version) {
        cameras.put(name, new LimelightCamera(networkTableName, version));
    }

    /**
     * Gets a camera by its friendly name
     * @param name The friendly name of the camera
     * @return The camera object if it exists
     */
    public Optional<LimelightCamera> getCamera(String name) {
        return Optional.ofNullable(cameras.get(name));
    }

    @Override
    public void periodic() {
        // Update all cameras
        cameras.values().forEach(LimelightCamera::updatePose);
    }
} 