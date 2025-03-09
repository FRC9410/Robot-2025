// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MapConstants;

public class Dashboard extends SubsystemBase {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final NetworkTable drivingTable;
  private ReefSide reefSide;
  private CoralSide coralSide;
  private CoralLevel coralLevel;
  private boolean isClimbing;
  private final BiConsumer<String, Object> updateData;
  
  /** Creates a new Dashboard. */
  public Dashboard(BiConsumer<String, Object> updateData) {
    this.updateData = updateData;
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Scoring");
    drivingTable = inst.getTable("Driving PIDs");

    table.getEntry("front").setBoolean(false);
    table.getEntry("front_left").setBoolean(false);
    table.getEntry("front_right").setBoolean(false);
    table.getEntry("back").setBoolean(false);
    table.getEntry("back_left").setBoolean(false);
    table.getEntry("back_right").setBoolean(false);
    table.getEntry("left").setBoolean(false);
    table.getEntry("right").setBoolean(false);
    table.getEntry("l1").setBoolean(false);
    table.getEntry("l2").setBoolean(false);
    table.getEntry("l3").setBoolean(false);
    table.getEntry("l4").setBoolean(false);
    table.getEntry("isClimbing").setBoolean(false);

    reefSide = null;
    coralSide = null;
    coralLevel = null;
  }

  @Override
  public void periodic() {
    final ReefSide newReefSide = getReefSide();
    if (newReefSide != reefSide) {
      setReefSide(newReefSide);
    }

    final CoralSide newCoralSide = getCoralSide();
    if (newCoralSide != coralSide) {
      setCoralSide(newCoralSide);
    }

    final CoralLevel newCoralLevel = getCoralLevel();
    if (newCoralLevel != coralLevel) {
      setCoralLevel(newCoralLevel);
    }

    isClimbing = table.getEntry("isClimbing").getBoolean(false);

    updateData.accept(MapConstants.TARGET_POSE, getScoringPose());
    updateData.accept(MapConstants.ELEVATOR_POSITION, getSelectedCoralLevel());
  }

  public ReefSide getReefSide() {
    if (table.getEntry("front").getBoolean(false) && reefSide != ReefSide.FRONT) {
      return ReefSide.FRONT;
    } else if (table.getEntry("front_left").getBoolean(false) && reefSide != ReefSide.FRONT_LEFT) {
      return ReefSide.FRONT_LEFT;
    } else if (table.getEntry("front_right").getBoolean(false) && reefSide != ReefSide.FRONT_RIGHT) {
      return ReefSide.FRONT_RIGHT;
    } else if (table.getEntry("back").getBoolean(false) && reefSide != ReefSide.BACK) {
      return ReefSide.BACK;
    } else if (table.getEntry("back_left").getBoolean(false) && reefSide != ReefSide.BACK_LEFT) {
      return ReefSide.BACK_LEFT;
    } else if (table.getEntry("back_right").getBoolean(false) && reefSide != ReefSide.BACK_RIGHT) {
      return ReefSide.BACK_RIGHT;
    }
    return reefSide;
  }

  public CoralSide getCoralSide() {
    if (table.getEntry("left").getBoolean(false) && coralSide != CoralSide.LEFT) {
      return CoralSide.LEFT;
    } else if (table.getEntry("right").getBoolean(false) && coralSide != CoralSide.RIGHT) {
      return CoralSide.RIGHT;
    }
    return coralSide;
  }

  public CoralLevel getCoralLevel() {
    if (table.getEntry("l1").getBoolean(false) && coralLevel != CoralLevel.L1) {
      return CoralLevel.L1;
    } else if (table.getEntry("l2").getBoolean(false) && coralLevel != CoralLevel.L2) {
      return CoralLevel.L2;
    } else if (table.getEntry("l3").getBoolean(false) && coralLevel != CoralLevel.L3) {
      return CoralLevel.L3;
    } else if (table.getEntry("l4").getBoolean(false) && coralLevel != CoralLevel.L4) {
      return CoralLevel.L4;
    }
    return coralLevel;
  }

  public boolean getIsClimbing () {
    return isClimbing;
  }

  public void clearSelections() {
    table.getEntry("front").setBoolean(false);
    table.getEntry("front_left").setBoolean(false);
    table.getEntry("front_right").setBoolean(false);
    table.getEntry("back").setBoolean(false);
    table.getEntry("back_left").setBoolean(false);
    table.getEntry("back_right").setBoolean(false);
    table.getEntry("left").setBoolean(false);
    table.getEntry("right").setBoolean(false);
    table.getEntry("l1").setBoolean(false);
    table.getEntry("l2").setBoolean(false);
    table.getEntry("l3").setBoolean(false);
    table.getEntry("l4").setBoolean(false);
    table.getEntry("isRed").setBoolean(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    reefSide = null;
    coralSide = null;
    coralLevel = null;
  }

  public void clearReefSelections() {
    table.getEntry("front").setBoolean(false);
    table.getEntry("front_left").setBoolean(false);
    table.getEntry("front_right").setBoolean(false);
    table.getEntry("back").setBoolean(false);
    table.getEntry("back_left").setBoolean(false);
    table.getEntry("back_right").setBoolean(false);
  }

  public void clearCoralLevelSelections() {
    table.getEntry("l1").setBoolean(false);
    table.getEntry("l2").setBoolean(false);
    table.getEntry("l3").setBoolean(false);
    table.getEntry("l4").setBoolean(false);
  }

  public void clearCoralSideSelections() {
    table.getEntry("left").setBoolean(false);
    table.getEntry("right").setBoolean(false);
  }

  public void setReefSide(ReefSide side) {
    if (side == null) {
      return;
    }

    clearReefSelections();
    reefSide = side;
    switch (side) {
      case FRONT:
        table.getEntry("front").setBoolean(true);
        break;
      case FRONT_LEFT:
        table.getEntry("front_left").setBoolean(true);
        break;
      case FRONT_RIGHT:
        table.getEntry("front_right").setBoolean(true);
        break;
      case BACK:
        table.getEntry("back").setBoolean(true);
        break;
      case BACK_LEFT:
        table.getEntry("back_left").setBoolean(true);
        break;
      case BACK_RIGHT:
        table.getEntry("back_right").setBoolean(true);
        break;
    }
  }

  public void setCoralSide( CoralSide side) {
    if (side == null) {
      return;
    }

    clearCoralSideSelections();
    coralSide = side;
    switch (side) {
      case LEFT:
        table.getEntry("left").setBoolean(true);
        break;
      case RIGHT:
        table.getEntry("right").setBoolean(true);
        break;
    }
  }

  public void setCoralLevel(CoralLevel level) {
    if (level == null) {
      return;
    }

    clearCoralLevelSelections();
    coralLevel = level;
    switch (level) {
      case L1:
        table.getEntry("l1").setBoolean(true);
        break;
      case L2:
        table.getEntry("l2").setBoolean(true);
        break;
      case L3:
        table.getEntry("l3").setBoolean(true);
        break;
      case L4:
        table.getEntry("l4").setBoolean(true);
        break;
    }
  }

  public Pose2d getScoringPose () {
    final boolean isRed = drivingTable.getEntry("invert controls").getBoolean(false);


    if (reefSide == null || coralSide == null) {
      return null;
    }

    if (isRed) {
      if (coralSide == CoralSide.LEFT) {
        switch (reefSide) {
          case FRONT:
            return Constants.ScoringConstants.RED_FRONT_LEFT;
          case FRONT_LEFT:
            return Constants.ScoringConstants.RED_FRONT_LEFT_LEFT;
          case FRONT_RIGHT:
            return Constants.ScoringConstants.RED_FRONT_RIGHT_LEFT;
          case BACK:
            return Constants.ScoringConstants.RED_BACK_LEFT;
          case BACK_LEFT:
            return Constants.ScoringConstants.RED_BACK_LEFT_LEFT;
          case BACK_RIGHT:
            return Constants.ScoringConstants.RED_BACK_RIGHT_LEFT;
          default:
            return null;
        }
      } else {
        switch (reefSide) {
          case FRONT:
            return Constants.ScoringConstants.RED_FRONT_RIGHT;
          case FRONT_LEFT:
            return Constants.ScoringConstants.RED_FRONT_LEFT_RIGHT;
          case FRONT_RIGHT:
            return Constants.ScoringConstants.RED_FRONT_RIGHT_RIGHT;
          case BACK:
            return Constants.ScoringConstants.RED_BACK_RIGHT;
          case BACK_LEFT:
            return Constants.ScoringConstants.RED_BACK_LEFT_RIGHT;
          case BACK_RIGHT:
            return Constants.ScoringConstants.RED_BACK_RIGHT_RIGHT;
          default:
            return null;
        }
      }
    } else {
      if (coralSide == CoralSide.LEFT) {
        switch (reefSide) {
          case FRONT:
            return Constants.ScoringConstants.BLUE_FRONT_LEFT;
          case FRONT_LEFT:
            return Constants.ScoringConstants.BLUE_FRONT_LEFT_LEFT;
          case FRONT_RIGHT:
            return Constants.ScoringConstants.BLUE_FRONT_RIGHT_LEFT;
          case BACK:
            return Constants.ScoringConstants.BLUE_BACK_LEFT;
          case BACK_LEFT:
            return Constants.ScoringConstants.BLUE_BACK_LEFT_LEFT;
          case BACK_RIGHT:
            return Constants.ScoringConstants.BLUE_BACK_RIGHT_LEFT;
          default:
            return null;
        }
      } else {
        switch (reefSide) {
          case FRONT:
            return Constants.ScoringConstants.BLUE_FRONT_RIGHT;
          case FRONT_LEFT:
            return Constants.ScoringConstants.BLUE_FRONT_LEFT_RIGHT;
          case FRONT_RIGHT:
            return Constants.ScoringConstants.BLUE_FRONT_RIGHT_RIGHT;
          case BACK:
            return Constants.ScoringConstants.BLUE_BACK_RIGHT;
          case BACK_LEFT:
            return Constants.ScoringConstants.BLUE_BACK_LEFT_RIGHT;
          case BACK_RIGHT:
            return Constants.ScoringConstants.BLUE_BACK_RIGHT_RIGHT;
          default:
            return null;
        }
      }
    }
  }

  public CoralSide getSelectedCoralSide() {
    return coralSide;
  }

  public double getSelectedCoralLevel() {
    if (coralLevel == null) {
      return Constants.ElevatorConstants.HOME_POSITION;
    }

    switch (coralLevel) {
      case L1:
        return Constants.ElevatorConstants.L1_SCORE_POSITION;
      case L2:
        return Constants.ElevatorConstants.L2_SCORE_POSITION;
      case L3:
        return Constants.ElevatorConstants.L3_SCORE_POSITION;
      case L4:
        return Constants.ElevatorConstants.L4_SCORE_POSITION;
      default:
        return Constants.ElevatorConstants.HOME_POSITION;
    }
  }

  public enum ReefSide {
    FRONT,
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK,
    BACK_LEFT,
    BACK_RIGHT
  }

  public enum CoralSide {
    LEFT,
    RIGHT
  }

  public enum CoralLevel {
    L1,
    L2,
    L3,
    L4
  }
}
