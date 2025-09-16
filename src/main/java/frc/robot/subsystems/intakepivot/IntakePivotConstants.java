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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class IntakePivotConstants {
  public static final int pivotMotorId = 42;

  public static final double pivotGearRatio =
      1; // idk this for now but im going to include it just in case
  public static final boolean kUseFOC = false; // do we need this??????
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
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(pivotGearRatio));


  public static final class PivotSim {
    // not sure about what gearing means and not sure if its equal to gear ratio
    public static final double pivotSimGearing = 10;

    public static final Distance intakePivotLength = Inches.of(0);
    public static final Mass intakePivotMass = Kilograms.of(2);
    public static final double jkGMetersSquared = 0;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(145.138);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(0.0);
  }
}
