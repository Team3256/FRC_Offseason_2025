// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

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

public class EndEffectorIOSim extends EndEffectorIOTalonFX {

  private final FlywheelSim eeSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              EndEffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
              EndEffectorConstants.SimulationConstants.eeGearingRatio,
              EndEffectorConstants.SimulationConstants.eeMomentOfInertia),
          EndEffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));
  private final TalonFXSimState eeMotorSim;

  private final CANrangeSimState eeCanRangeSim;

  private final LoggedTunableNumber canRangeDistance =
      new LoggedTunableNumber("EECanRangeDistance", 0.0);

  public EndEffectorIOSim() {
    super();

    eeMotorSim = super.getEEMotor().getSimState();
    eeCanRangeSim = super.getCanRange().getSimState();


  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {


    // Update battery voltage
    eeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    eeCanRangeSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    eeCanRangeSim.setDistance(canRangeDistance.getAsDouble());

    // Update physics models
    eeSimModel.setInput(eeMotorSim.getMotorVoltage());
    eeSimModel.update(LoggedRobot.defaultPeriodSecs);

    double eeRps = eeSimModel.getAngularVelocityRPM() / 60;
    eeMotorSim.setRotorVelocity(eeRps);
    eeMotorSim.addRotorPosition(eeRps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            eeSimModel.getCurrentDrawAmps(), eeSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateEndEffector(
            Degrees.of(
                Math.toDegrees(eeRps)
                    * LoggedRobot.defaultPeriodSecs
                    * EndEffectorConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
