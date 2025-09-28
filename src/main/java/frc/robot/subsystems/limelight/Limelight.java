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

    private final String name;

    public Limelight(String name) {
        this.name = name;
    }





  public  double getTX() {
    return LimelightHelpers.getTX(name);
  }

  public  double getTY() {
    return LimelightHelpers.getTY(name);
  }

}
