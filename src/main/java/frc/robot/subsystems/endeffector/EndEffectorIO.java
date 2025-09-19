// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {

    public double eeMotorVoltage = 0;
    public double eeMotorVelocity = 0;
    public double eeMotorStatorCurrent = 0;
    public double eeMotorSupplyCurrent = 0;
    public double canRangeDistance = 0.0;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEEVoltage(double voltage) {}

  public default void setEEVelocity(double velocity) {}

  public default TalonFX getEEMotor() {
    return new TalonFX(0);
  }

  public default CANrange getCanRange() {
    return new CANrange(0);
  }

  public default void eeOff() {}
}
