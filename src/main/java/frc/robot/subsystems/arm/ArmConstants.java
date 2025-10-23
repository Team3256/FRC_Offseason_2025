// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ArmConstants {

  public static final int armMotorId = 42;

  public static final int armMotorEncoderId = 0;

  // max value is 8, min is 0

  /* Misc */
  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true; // idk
  public static final int flashConfigRetries = 5;

  public static final Angle maxRotations = Rotations.of(1.5);

  public static final double deadband = 0.11;

  // Arm positions
  // L1, L2-L3 (since same arm angle), L4
  // "arm level 0" is L1, "arm level 1" is L2-L3, "arm level 2" is L4
  // 3 sig figs of acc
  public static final Angle[] reefRightPrepPositions = {
    Rotations.of(0.3), Rotations.of(0.3), Rotations.of(0.343)
  };

  // @deprecated
  public static final Angle[] reefLeftPrepPositions = {
    Rotations.of(0.2), Rotations.of(0.2), Rotations.of(0.157)
  };

  public static final Angle[] reefRightScoringPosition = {
    Rotations.of(.4), Rotations.of(.5), Rotations.of(.55)
  };
  public static final Angle[] reefLeftScoringPosition = {
    Rotations.of(.1), Rotations.of(0), Rotations.of(0.95)
  };

  // Dealgae L2, Daalgae L3
  public static final Angle[] dealgaeRightPosition = {Rotations.of(0.5), Rotations.of(.5)};
  public static final Angle[] dealgaeLeftPosition = {Rotations.of(0), Rotations.of(0)};

  public static final Angle sourcePosition = Rotations.of(.285);

  public static final Angle bargeLeftPosition = Rotations.of(.17);
  public static final Angle bargeRightPosition = Rotations.of(.33);

  public static final Angle prebargeLeftPosition = Rotations.of(.33);
  public static final Angle prebargeRightPosition = Rotations.of(.17);

  public static final Angle homePosition = Rotations.of(.25);

  public static final double safeRightPosition = .65;
  public static final double safeLeftPosition = 0.85;

  public static final double handoffPosition = 0.75;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(.0996)
                  .withKV(8.9964)
                  .withKP(80)
                  .withKI(0)
                  .withKD(1)
                  .withKA(0)
                  .withKG(.3)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicJerk(40)
                  .withMotionMagicAcceleration(7.5)
                  .withMotionMagicCruiseVelocity(1.2))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
                  .withFeedbackRemoteSensorID(armMotorEncoderId)
                  .withSensorToMechanismRatio(4 / 3.0)
                  .withRotorToSensorRatio(48));

  //  public static final TalonFXConfiguration simMotorConfigs =
  //      motorConfigs.withFeedback(
  //          new FeedbackConfigs()
  //              .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
  //              .withSensorToMechanismRatio(142.22));

  public static final CANcoderConfiguration cancoderConfiguration =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withMagnetOffset(0)
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));
  public static final Angle processorRightPosition = Rotations.of(.576);

  public static final Angle groundAlgaeRightPosition = Rotations.of(.62);

  public static final Angle armAngleCoralLeft = Rotations.of(0.76);
  public static final Angle armAngleCoralRight = Rotations.of(0.72);
  public static final double coralDistanceLeft = 0.07; // inches
  public static final double coralDistanceRight = .2; // inches

  public static final class Sim {
    public static final double simGearing = 142.22;

    public static final Distance armLength = Inches.of(22);
    public static final Mass armMass = Kilograms.of(2);
    public static final double jkGMetersSquared = 1.2922967095;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(-720);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(720);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(90);
  }
}
