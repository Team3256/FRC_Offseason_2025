// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;

public class IntakePivot extends DisableSubsystem {

  private final IntakePivotIO intakePivotIO;
  private final IntakePivotIOInputsAutoLogged intakePivotIOInputsAutoLogged =
      new IntakePivotIOInputsAutoLogged();

  private double reqPosition = 0.0;

  public IntakePivot(boolean enabled, IntakePivotIO intakePivotIO) {
    super(enabled);

    this.intakePivotIO = intakePivotIO;

    this.intakePivotIO.resetPosition(Rotations.of(.427));
  }

  @Override
  public void periodic() {
    super.periodic();
    intakePivotIO.updateInputs(intakePivotIOInputsAutoLogged);
    Logger.processInputs("IntakePivot", intakePivotIOInputsAutoLogged);

    Logger.recordOutput(this.getClass().getSimpleName() + "/reqPosition", reqPosition);

    LoggedTracer.record("IntakePivot");
  }

  public Command setPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          reqPosition = position.getAsDouble();
          intakePivotIO.setPosition(reqPosition);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> intakePivotIO.setVoltage(voltage));
  }

  public Command zero() {
    return this.runOnce(intakePivotIO::zero);
  }

  public Command off() {
    return this.runOnce(intakePivotIO::off).withName("off");
  }

  public Command goToStow() {
    return this.setPosition(IntakePivotConstants.stowPosition);
  }

  public Command goToHandoff() {
    return this.setPosition(IntakePivotConstants.handoffPosition);
  }

  public Command goToGroundIntake() {
    return this.setPosition(IntakePivotConstants.groundIntakePosition);
  }
}
