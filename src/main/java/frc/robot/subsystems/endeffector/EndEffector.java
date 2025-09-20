// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends DisableSubsystem {

  private final EndEffectorIO endEffectorIO;
  private final EndEffectorIOInputsAutoLogged endEffectorIOInputsAutoLogged =
      new EndEffectorIOInputsAutoLogged();

  public final Trigger gamePieceIntaken =
      new Trigger(
          () ->
              endEffectorIOInputsAutoLogged.canRangeDistance
                  < EndEffectorConstants.gamePieceIntakenIn);

  public EndEffector(boolean enabled, EndEffectorIO endEffectorIO) {
    super(enabled);
    this.endEffectorIO = endEffectorIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    endEffectorIO.updateInputs(endEffectorIOInputsAutoLogged);
    Logger.processInputs("EndEffector", endEffectorIOInputsAutoLogged);

    LoggedTracer.record("EndEffector");
  }

  public Command setEEVoltage(DoubleSupplier voltage) {
    return this.run(() -> endEffectorIO.setEEVoltage(voltage.getAsDouble()));
  }

  public Command setEEVelocity(DoubleSupplier velocity) {
    return this.run(() -> endEffectorIO.setEEVelocity(velocity.getAsDouble()));
  }

  public Command setCoralOuttakeVoltage() {
    return setEEVoltage(() -> EndEffectorConstants.coralOuttakeVoltage)
        .withName("setCoralOuttakeVoltage");
  }

  public Command setAlgaeIntakeVoltage() {
    return setEEVoltage(() -> EndEffectorConstants.algaeIntakeVoltage)
        .withName("setAlgaeIntakeVoltage");
  }

  public Command setAlgaeOuttakeVoltage() {
    return setEEVoltage(() -> EndEffectorConstants.algaeOuttakeVoltage)
        .withName("setAlgaeOuttakeVoltage");
  }

  public Command setSourceVelocity() {
    return setEEVelocity(() -> EndEffectorConstants.sourceVelocityRps)
        .withName("setSourceVelocity");
  }

  public Command off() {
    return this.runOnce(endEffectorIO::eeOff).withName("eeOff");
  }

  public Command intakeCoral() {
    return setEEVoltage(() -> EndEffectorConstants.coralIntakeVoltage)
        .until(gamePieceIntaken)
        .finallyDo(endEffectorIO::eeOff)
        .withName("setCoralIntakeVoltage");
  }
}
