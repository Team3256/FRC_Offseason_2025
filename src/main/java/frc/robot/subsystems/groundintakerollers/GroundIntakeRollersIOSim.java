// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.groundintakerollers;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.endeffector.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;


import org.littletonrobotics.junction.LoggedRobot;

public class GroundIntakeRollersIOSim extends GroundIntakeRollersIOTalonFX {
public class EndEffectorIOSim extends EndEffectorIOTalonFX {
  private final FlywheelSim rollerSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              GroundIntakeRollersConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
              GroundIntakeRollersConstants.SimulationConstants.rollerGearingRatio,
              GroundIntakeRollersConstants.SimulationConstants.rollerMomentOfInertia),
              GroundIntakeRollersConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));

private final TalonFXSimState motorSim;


private final CANdiSimState candiSim;

  private final SendableChooser<Boolean> s1Closed =
      new SendableChooser<>() {
        {
          addOption("Algae Inside", true);
          addOption("No Algae", false);
        }
      };



  public GroundIntakeRollersIOSim() {
    super();
    motorSim = super.getAlgaeMotor().getSimState();
    candiSim = super.getCandi().getSimState();
    s1Closed.onChange(this::updateCandiS1);
    s1Closed.setDefaultOption("No Algae", false);
    SmartDashboard.putData("S1", s1Closed);
  }

  private void updateCandiS1(boolean beamBroken) {
    candiSim.setS1State(beamBroken ? S1StateValue.Floating : S1StateValue.Low);
  }


  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {

    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    candiSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Update physics models
    rollerSimModel.setInput(rollerSimModel.getMotorVoltage());
    rollerSimModel.update(LoggedRobot.defaultPeriodSecs);
  

    double algaeRps = rollerSimModel.getAngularVelocityRPM() / 60;
    motorSim.setRotorVelocity(algaeRps);
    motorSim.addRotorPosition(algaeRps * LoggedRobot.defaultPeriodSecs);
  

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
          rollerSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateEndEffector(
            Degrees.of(
                Math.toDegrees(algaeRps)
                    * LoggedRobot.defaultPeriodSecs
                    * GroundIntakeRollersConstants.SimulationConstants.kAngularVelocityScalar));
        
}}}