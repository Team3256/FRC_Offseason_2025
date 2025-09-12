// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX pivotMotor = new TalonFX(IntakePivotConstants.pivotMotorId);
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(IntakePivotConstants.kUseFOC);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(IntakePivotConstants.kUseFOC);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> pivotMotorVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Angle> pivotMotorPosition = pivotMotor.getPosition();
  private final StatusSignal<Current> pivotMotorStatorCurrent = pivotMotor.getStatorCurrent();
  private final StatusSignal<Current> pivotMotorSupplyCurrent = pivotMotor.getSupplyCurrent();


  public IntakePivotIOTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        pivotMotor, IntakePivotConstants.motorConfigs, IntakePivotConstants.flashConfigRetries);

    PhoenixUtil.registerSignals(
        false,
        pivotMotorVoltage,
        pivotMotorVelocity,
        pivotMotorPosition,
        pivotMotorStatorCurrent,
        pivotMotorSupplyCurrent);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.pivotMotorVoltage = pivotMotorVoltage.getValue().in(Volt);
    inputs.pivotMotorVelocity = pivotMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.pivotMotorPosition = pivotMotorPosition.getValue().in(Rotations);
    inputs.pivotMotorStatorCurrent = pivotMotorStatorCurrent.getValue().in(Amps);
    inputs.pivotMotorSupplyCurrent = pivotMotorSupplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(Angle position) {
    if (IntakePivotConstants.kUseMotionMagic) {
      pivotMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      pivotMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setPosition(double position) {
    if (IntakePivotConstants.kUseMotionMagic) {
      pivotMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      pivotMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(Voltage voltage) {
    pivotMotor.setVoltage(voltage.in(Volt));
  }

  @Override
  public void off() {
    pivotMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return pivotMotor;
  }

  @Override
  public void resetPosition(Angle angle) {
    pivotMotor.setPosition(angle);
  }
}