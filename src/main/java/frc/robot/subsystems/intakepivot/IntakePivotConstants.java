// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

public final class IntakePivotConstants {
  public static final int pivotMotorId = 42;

  public static final double pivotGearRatio = 0; // idk this for now but im going to include it just in case 
  public static final boolean kUseFOC = false; //do we need this??????
  public static final boolean kUseMotionMagic = true; // idk if pivot needs motion magic
  public static final int flashConfigRetries = 5;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(0)
                  .withKI(0)
                  .withKD(0)
                  .withKA(0)
                  .withKG(.35))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(3.5)
                  .withMotionMagicCruiseVelocity(.6))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(1.33333)
                  .withRotorToSensorRatio(69.9999975));

  public static final CANcoderConfiguration cancoderConfiguration =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withMagnetOffset(-0.6601640625)
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));

}