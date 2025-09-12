// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.groundintakerollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeRollers extends DisableSubsystem {
  private final GroundIntakeRollersIO groundIntakeRollersIO;
  private final GroundIntakeRollersIOInputsAutoLogged intakeIOAutoLogged =
      new GroundIntakeRollersIOInputsAutoLogged();

  public final Trigger motorStalled =
      new Trigger(
          () ->
              (intakeIOAutoLogged.intakeRollerMotorStatorCurrent
                  > GroundIntakeRollersConstants.motorCoralStall));

  public final Trigger coralIntakeIn =
      new Trigger(
          () -> (intakeIOAutoLogged.canRangeDistance < GroundIntakeRollersConstants.coralIntakeIn));

  public GroundIntakeRollers(boolean disabled, GroundIntakeRollersIO groundIntakeRollersIO) {
    super(disabled);
    this.groundIntakeRollersIO = groundIntakeRollersIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    groundIntakeRollersIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs("GroundIntake", intakeIOAutoLogged);

    LoggedTracer.record("GroundIntake");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> groundIntakeRollersIO.setIntakeRollerVoltage(voltage))
        .finallyDo(groundIntakeRollersIO::off);
  }

  public Command setVelocity(double velocity) {
    return this.run(() -> groundIntakeRollersIO.setIntakeRollerVelocity(velocity))
        .finallyDo(groundIntakeRollersIO::off);
  }

  public Command off() {
    return this.runOnce(groundIntakeRollersIO::off);
  }

  public Command intakeCoral() {
    return setVoltage(GroundIntakeRollersConstants.kIntakeRollerMotorVoltage)
        .until(motorStalled)
        .finallyDo(groundIntakeRollersIO::off);
  }
}
