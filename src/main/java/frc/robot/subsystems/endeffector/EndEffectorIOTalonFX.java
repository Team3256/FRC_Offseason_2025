// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX coralMotor = new TalonFX(EndEffectorConstants.coralMotorID);
  final VelocityVoltage coralVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final VoltageOut coralVoltageRequest =
      new VoltageOut(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final StatusSignal<Voltage> coralMotorVoltage = coralMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> coralMotorVelocity = coralMotor.getVelocity();
  private final StatusSignal<Current> coralMotorStatorCurrent = coralMotor.getStatorCurrent();
  private final StatusSignal<Current> coralMotorSupplyCurrent = coralMotor.getSupplyCurrent();

  private final CANdi candi = new CANdi(EndEffectorConstants.candiID);

  private final StatusSignal<Boolean> coralBeamBreak = candi.getS2Closed();

  public EndEffectorIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        coralMotor,
        EndEffectorConstants.coralMotorConfigs,
        EndEffectorConstants.flashConfigRetries);

    PhoenixUtil.applyCANdiConfigs(
        candi, EndEffectorConstants.canDiConfigs, EndEffectorConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        EndEffectorConstants.updateFrequency,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent,
        coralBeamBreak);
    PhoenixUtil.registerSignals(
        false,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent,
        coralBeamBreak);

    coralMotor.optimizeBusUtilization(4, 0.100);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.coralMotorVoltage = coralMotorVoltage.getValueAsDouble();
    inputs.coralMotorVelocity = coralMotorVelocity.getValueAsDouble();
    inputs.coralMotorStatorCurrent = coralMotorStatorCurrent.getValueAsDouble();
    inputs.coralMotorSupplyCurrent = coralMotorSupplyCurrent.getValueAsDouble();

    inputs.coralBeamBreak = coralBeamBreak.getValue();
  }

  @Override
  public void setCoralVoltage(double voltage) {
    setCoralVoltage(voltage, false);
  }

  @Override
  public void setCoralVelocity(AngularVelocity velocity) {
    setCoralVelocity(velocity, false);
  }

  @Override
  public void setCoralVoltage(double voltage, boolean override) {
    coralMotor.setControl(
        coralVoltageRequest.withOutput(Volts.of(voltage)).withIgnoreHardwareLimits(override));
  }

  @Override
  public void setCoralVelocity(AngularVelocity velocity, boolean override) {
    coralMotor.setControl(
        coralVelocityRequest.withVelocity(velocity).withIgnoreHardwareLimits(override));
  }

  @Override
  public void coralOff() {
    coralMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getCoralMotor() {
    return coralMotor;
  }

  @Override
  public CANdi getCandi() {
    return candi;
  }
}
