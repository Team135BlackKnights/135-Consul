package frc.robot.utils.CompetitionFieldUtils.ReferencePoses;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public final class FieldReferencePoseLogger {
  private FieldReferencePoseLogger() {
  }

  public static void logIfEnabled() {
    if (!Constants.logFieldReferencePoses) {
      return;
    }

    Logger.recordOutput("Field/ReferencePoses/Hub/Blue/Center", FieldReferencePoses.BLUE_HUB_CENTER);
    Logger.recordOutput("Field/ReferencePoses/Hub/Blue/Corners", FieldReferencePoses.BLUE_HUB_CORNERS);
    Logger.recordOutput("Field/ReferencePoses/Hub/Red/Center", FieldReferencePoses.RED_HUB_CENTER);
    Logger.recordOutput("Field/ReferencePoses/Hub/Red/Corners", FieldReferencePoses.RED_HUB_CORNERS);

    Logger.recordOutput("Field/ReferencePoses/Tower/Blue/Center", FieldReferencePoses.BLUE_TOWER_CENTER);
    Logger.recordOutput("Field/ReferencePoses/Tower/Blue/Uprights", FieldReferencePoses.BLUE_TOWER_UPRIGHTS);
    Logger.recordOutput("Field/ReferencePoses/Tower/Blue/NoGoCorners", FieldReferencePoses.BLUE_TOWER_NO_GO_CORNERS);
    Logger.recordOutput("Field/ReferencePoses/Tower/Red/Center", FieldReferencePoses.RED_TOWER_CENTER);
    Logger.recordOutput("Field/ReferencePoses/Tower/Red/Uprights", FieldReferencePoses.RED_TOWER_UPRIGHTS);
    Logger.recordOutput("Field/ReferencePoses/Tower/Red/NoGoCorners", FieldReferencePoses.RED_TOWER_NO_GO_CORNERS);

    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/BlueBottom/OpeningCenter",
        FieldReferencePoses.BLUE_BOTTOM_TRENCH_OPENING_CENTER);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/BlueBottom/Corners",
        FieldReferencePoses.BLUE_BOTTOM_TRENCH_WALL_CORNERS);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/BlueTop/OpeningCenter",
        FieldReferencePoses.BLUE_TOP_TRENCH_OPENING_CENTER);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/BlueTop/Corners",
        FieldReferencePoses.BLUE_TOP_TRENCH_WALL_CORNERS);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/RedBottom/OpeningCenter",
        FieldReferencePoses.RED_BOTTOM_TRENCH_OPENING_CENTER);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/RedBottom/Corners",
        FieldReferencePoses.RED_BOTTOM_TRENCH_WALL_CORNERS);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/RedTop/OpeningCenter",
        FieldReferencePoses.RED_TOP_TRENCH_OPENING_CENTER);
    Logger.recordOutput(
        "Field/ReferencePoses/TrenchWalls/RedTop/Corners",
        FieldReferencePoses.RED_TOP_TRENCH_WALL_CORNERS);
  }
}
