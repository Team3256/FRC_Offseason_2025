// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public class IntakePivotIOInputs {

    public double pivotMotorVoltage = 0.0;
    public double pivotMotorVelocity = 0.0;
    public double pivotMotorPosition = 0.0;
    public double pivotMotorStatorCurrent = 0.0;
    public double pivotMotorSupplyCurrent = 0.0;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPosition(Angle position) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default void resetPosition(Angle angle) {}

  public default void off() {}

  public default void zero() {}
}
