// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

public final class EndEffectorConstants {

  public static final double gamePieceIntakenIn = 0.2;
  public static final boolean kUseFOC = true;

  // Constants used in CANrange construction
  public static final int kCANrangeId = 0;

  // Configure the CANrange for basic use
  public static final CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();

  public static final int coralMotorID = 43;

  public static final double coralOuttakeVoltage = 3;
  public static final double coralIntakeVoltage = 12;

  // algae first then coral
  public static final double sourceVelocityRps = 30;

  public static TalonFXConfiguration coralMotorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0).withKV(.118).withKA(0).withKP(.2).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerTime(1)
                  .withSupplyCurrentLowerLimit(40));

  public static final double stallStatorCurrent = 50;
  public static final double algaeIntakeVoltage = 3;
  public static double algaeOuttakeVoltage = 4;

  public static final class SimulationConstants {

    public static double eeGearingRatio = 1.0;
    public static double eeMomentOfInertia = 1;
    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 5;
  }

  public static double updateFrequency = 50.0;

  public static int flashConfigRetries = 5;
}
