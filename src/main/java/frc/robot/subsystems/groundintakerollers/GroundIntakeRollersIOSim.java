// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.groundintakerollers;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.LoggedRobot;

public class GroundIntakeRollersIOSim extends GroundIntakeRollersIOTalonFX {
  private final FlywheelSim rollerSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              GroundIntakeRollersConstants.kUseFOC
                  ? DCMotor.getKrakenX60Foc(1)
                  : DCMotor.getKrakenX60(1),
              GroundIntakeRollersConstants.SimulationConstants.rollerGearingRatio,
              GroundIntakeRollersConstants.SimulationConstants.rollerMomentOfInertia),
          GroundIntakeRollersConstants.kUseFOC
              ? DCMotor.getKrakenX60Foc(1)
              : DCMotor.getKrakenX60(1));

  private final TalonFXSimState motorSim;

  private final CANrangeSimState canRangeSimState;

  private final LoggedTunableNumber canRangeDistance =
      new LoggedTunableNumber("GIRCanRangeDistance", 0.0);

  public GroundIntakeRollersIOSim() {
    super();
    motorSim = super.getIntakeRollerMotor().getSimState();
    canRangeSimState = super.getCanRange().getSimState();

    LoggedTunableNumber.ifChanged(
        canRangeDistance.hashCode(), this::updateCanRangeDistance, canRangeDistance);
  }

  public void updateCanRangeDistance(double[] distanceMeters) {
    canRangeSimState.setDistance(distanceMeters[0]);
  }

  @Override
  public void updateInputs(GroundIntakeRollersIOInputs inputs) {

    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    canRangeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics models
    rollerSimModel.setInput(motorSim.getMotorVoltage());
    rollerSimModel.update(LoggedRobot.defaultPeriodSecs);

    double motorRPS = rollerSimModel.getAngularVelocityRPM() / 60;
    motorSim.setRotorVelocity(motorRPS);
    motorSim.addRotorPosition(motorRPS * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateRollers(
            Degrees.of(
                Math.toDegrees(motorRPS)
                    * LoggedRobot.defaultPeriodSecs
                    * GroundIntakeRollersConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
