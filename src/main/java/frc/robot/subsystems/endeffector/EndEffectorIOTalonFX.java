// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX coralMotor = new TalonFX(EndEffectorConstants.coralMotorID);
  private final CANrange canRange = new CANrange(EndEffectorConstants.kCANrangeId);
  final VelocityVoltage coralVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final VoltageOut coralVoltageRequest =
      new VoltageOut(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final StatusSignal<Voltage> coralMotorVoltage = coralMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> coralMotorVelocity = coralMotor.getVelocity();
  private final StatusSignal<Current> coralMotorStatorCurrent = coralMotor.getStatorCurrent();
  private final StatusSignal<Current> coralMotorSupplyCurrent = coralMotor.getSupplyCurrent();
  private final StatusSignal<Distance> canRangeDistance = canRange.getDistance();

  public EndEffectorIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        coralMotor,
        EndEffectorConstants.coralMotorConfigs,
        EndEffectorConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        EndEffectorConstants.updateFrequency,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent,
        canRangeDistance);
    PhoenixUtil.registerSignals(
        false,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent,
        canRangeDistance);

    coralMotor.optimizeBusUtilization(4, 0.100);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.eeMotorVoltage = coralMotorVoltage.getValueAsDouble();
    inputs.eeMotorVelocity = coralMotorVelocity.getValueAsDouble();
    inputs.eeMotorStatorCurrent = coralMotorStatorCurrent.getValueAsDouble();
    inputs.eeMotorSupplyCurrent = coralMotorSupplyCurrent.getValueAsDouble();
    inputs.canRangeDistance = canRangeDistance.getValueAsDouble();
  }

  @Override
  public void setEEVoltage(double voltage) {
    coralMotor.setVoltage(voltage);
  }

  @Override
  public void setEEVelocity(double velocity) {
    coralMotor.setControl(coralVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void eeOff() {
    coralMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getEEMotor() {
    return coralMotor;
  }
}
