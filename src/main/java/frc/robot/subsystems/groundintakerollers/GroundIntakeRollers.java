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

public class GroundIntakeRollers extends DisableSubsystem {
  private final GroundIntakeRollersIO groundIntakeRollersIO;

  public GroundIntakeRollers(boolean disabled, GroundIntakeRollersIO groundIntakeRollersIO) {
    super(disabled);
    this.groundIntakeRollersIO = groundIntakeRollersIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    // add logging later??
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> groundIntakeRollersIO.setIntakeRollerVoltage(voltage))
        .finallyDo(groundIntakeRollersIO::off);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return this.run(() -> groundIntakeRollersIO.setIntakeRollerVelocity(velocity))
        .finallyDo(groundIntakeRollersIO::off);
  }

  public Command off() {
    return this.runOnce(groundIntakeRollersIO::off);
  }

  public Command intakeIn(Trigger canTrigger) {
    return this.run(
            () ->
                groundIntakeRollersIO.setIntakeRollerVoltage(
                    GroundIntakeRollersConstants.kIntakeRollerMotorVoltage))
        .until(canTrigger)
        .finallyDo(groundIntakeRollersIO::off);
  }
}
