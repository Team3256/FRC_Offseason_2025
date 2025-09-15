// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends DisableSubsystem {

  private final EndEffectorIO endEffectorIO;
  private final EndEffectorIOInputsAutoLogged endEffectorIOInputsAutoLogged =
      new EndEffectorIOInputsAutoLogged();

  public final Trigger coralBeamBreak =
      new Trigger(() -> endEffectorIOInputsAutoLogged.coralBeamBreak);

  public final Trigger algaeBeamBreak =
      new Trigger(() -> endEffectorIOInputsAutoLogged.coralBeamBreak);

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

  public Command setCoralVoltage(DoubleSupplier voltage) {
    return this.run(() -> endEffectorIO.setCoralVoltage(voltage.getAsDouble()));
  }

  public Command setCoralVelocity(Supplier<AngularVelocity> velocity) {
    return this.run(() -> endEffectorIO.setCoralVelocity(velocity.get()));
  }

  public Command setL1Velocity() {
    return this.setCoralVelocity(() -> EndEffectorConstants.l1Velocity);
  }

  public Command setL2L3Velocity() {
    return this.setCoralVelocity(() -> EndEffectorConstants.l2l3Velocity);
  }

  public Command setL4Voltage() {
    return this.setCoralVoltage(() -> EndEffectorConstants.l4Voltage);
  }

  public Command setSourceVelocity() {
    return setCoralVelocity(() -> EndEffectorConstants.sourceVelocity)
        .withName("setSourceVelocity");
  }

  public Command coralOff() {
    return this.runOnce(endEffectorIO::coralOff).withName("coralOff");
  }
}
