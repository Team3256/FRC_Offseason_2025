// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.groundintakerollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class GroundIntakeRollersIOTalonFX implements GroundIntakeRollersIO {
  private final TalonFX intakeMotor =
      new TalonFX(GroundIntakeRollersConstants.kIntakeRollerMotorID);
  private final CANrange canRange = new CANrange(GroundIntakeRollersConstants.kIntakeRollerMotorID);
  final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> intakeRollerMotorVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> intakeRollerMotorVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Current> intakeRollerMotorStatorCurrent =
      intakeMotor.getStatorCurrent();
  private final StatusSignal<Current> intakeRollerMotorSupplyCurrent =
      intakeMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> intakeRollerMotorTemperature =
      intakeMotor.getDeviceTemp();
  private final StatusSignal<Distance> canRangeDistance = canRange.getDistance();

  public GroundIntakeRollersIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        intakeMotor,
        GroundIntakeRollersConstants.motorConfigs,
        GroundIntakeRollersConstants.flashConfigRetries);
    PhoenixUtil.applyCanRangeConfigs(
        canRange,
        GroundIntakeRollersConstants.canRangeConfigs,
        GroundIntakeRollersConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        GroundIntakeRollersConstants.updateFrequency,
        intakeRollerMotorVoltage,
        intakeRollerMotorVelocity,
        intakeRollerMotorStatorCurrent,
        intakeRollerMotorSupplyCurrent,
        intakeRollerMotorTemperature,
        canRangeDistance);
    PhoenixUtil.registerSignals(
        false,
        intakeRollerMotorVoltage,
        intakeRollerMotorVelocity,
        intakeRollerMotorStatorCurrent,
        intakeRollerMotorSupplyCurrent,
        intakeRollerMotorTemperature,
        canRangeDistance);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GroundIntakeRollersIOInputs inputs) {

    inputs.intakeRollerMotorVoltage = intakeRollerMotorVoltage.getValueAsDouble();
    inputs.intakeRollerMotorVelocity = intakeRollerMotorVelocity.getValueAsDouble();
    inputs.intakeRollerMotorStatorCurrent = intakeRollerMotorStatorCurrent.getValueAsDouble();
    inputs.intakeRollerMotorSupplyCurrent = intakeRollerMotorSupplyCurrent.getValueAsDouble();
    inputs.intakeRollerMotorTemperature = intakeRollerMotorTemperature.getValueAsDouble();
    inputs.canRangeDistance = canRangeDistance.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setVelocity(double velocity) {

    intakeMotor.setControl(intakeRequest.withVelocity(velocity));
  }

  @Override
  public void off() {
    intakeMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getIntakeRollerMotor() {
    return intakeMotor;
  }
}
