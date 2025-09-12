// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.groundintakerollers;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GroundIntakeRollersConstants {

  // Constants used in CANrange construction
  public static final int kCANrangeId = 0;
  public static final String kCANrangeCANbus = "canivore";

  // Configure the CANrange for basic use
  public static final CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();

  public static final int kIntakeRollerMotorID = 46;
  public static final double kIntakeRollerMotorVoltage = 12;

  public static double updateFrequency = 50;
  public static boolean kIntakeMotionMagic = false;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0.1).withKP(1).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(120)
                  .withMotionMagicCruiseVelocity(60)
                  .withMotionMagicJerk(1200))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(120));
  public static int flashConfigRetries = 5;
  public static double kIntakeRedirectVoltage = 5;

  public static final int motorCoralStall = 40; 
  public static final double coralIntakeIn = 0.2;
}
