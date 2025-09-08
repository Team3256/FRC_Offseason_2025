// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.groundintakerollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public interface GroundIntakeRollersIO {
  public static class GroundIntakeIOInputs {
    public double intakeRollerMotorVoltage = 0.0;
    public double intakeRollerMotorVelocity = 0.0;
    public double intakeRollerMotorStatorCurrent = 0.0;
    public double intakeRollerMotorSupplyCurrent = 0.0;
    public double intakeRollerMotorTemperature = 0.0;
  }

  public default void updateInputs(GroundIntakeIOInputs inputs) {}

  public default void setIntakeRollerVoltage(double voltage) {}

  public default void setIntakeRollerVelocity(double velocity) {}

  public default TalonFX getIntakeRollerMotor() {
    return new TalonFX(0);
  }

  public default CANrange getCanRange() {
    return new CANrange(0);
  }

  public default void off() {}
}
