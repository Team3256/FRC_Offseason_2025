// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends DisableSubsystem {

  private final IntakePivotIO intakePivotIO;
  private final IntakePivotIOInputsAutoLogged intakePivotIOInputsAutoLogged = new IntakePivotIOInputsAutoLogged();

  //public final Trigger isDown = new Trigger(this::isDown);
  //public final Trigger isStowed = new Trigger(this::isStowed);

  public IntakePivot(boolean enabled, IntakePivotIO intakePivotIO) {
    super(enabled);

    this.intakePivotIO = intakePivotIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    intakePivotIO.updateInputs(intakePivotIOInputsAutoLogged);
    Logger.processInputs("IntakePivot", intakePivotIOInputsAutoLogged);

    LoggedTracer.record("IntakePivot");
  }


 public Command setPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          intakePivotIO.setPosition(position.getAsDouble());
        });
  }

  public double getPosition() {
    return intakePivotIOInputsAutoLogged.motorPosition;
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> intakePivotIO.setVoltage(voltage));
  }

  public Command zero() {
    return this.runOnce(intakePivotIO::zero);
  }

  public Command off() {
    return this.runOnce(intakePivotIO::off).withName("off");
  }

}
