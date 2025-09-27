// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.LimelightHelpers;

public class Limelight {

  static final String sanitizeName(String name) {
    if ("".equals(name) || name == null) {
      return "limelight";
    }
    return name;
  }

  public static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  public static double getTX(String limelightName) {
    return getLimelightNTDouble(limelightName, "tx");
  }

  public static double getTY(String limelightName) {
    return getLimelightNTDouble(limelightName, "ty");
  }

  double tx = LimelightHelpers.getTX("limelight");
  double ty = LimelightHelpers.getTY("limelight");
  // boolean hasTarget = LimelightHelpers.getTV("limelight");

}
