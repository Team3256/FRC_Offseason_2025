// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintakerollers.GroundIntakeRollers;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.utils.LoggedTracer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public enum StructureState {
    PREBARGE,
    BARGE,
    CANCEL_ALL,
    DEALGAE_L2,
    DEALGAE_L3,
    HOME,
    IDLE,
    L1,
    L2,
    L3,
    L4,
    PREHOME,
    PROCESSOR,
    SCORE_ALGAE,
    SCORE_CORAL,
    GROUND_INTAKE,
    PRE_HANDOFF,
    HANDOFF,
  }

  public enum ManipulatorSide {
    LEFT,
    RIGHT
  }

  private ManipulatorSide manipulatorSide = ManipulatorSide.RIGHT;

  private final Trigger rightManipulatorSide =
      new Trigger(() -> this.manipulatorSide == ManipulatorSide.RIGHT);

  private final Trigger reefOnRight;

  private final Trigger bargeOnRight;

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Map<StructureState, Trigger> prevStateTriggers = new HashMap<StructureState, Trigger>();

  private final Timer stateTimer = new Timer();

  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Arm arm;
  private final GroundIntakeRollers intakeRollers;
  private final IntakePivot intakePivot;

  private final Supplier<Pose2d> robotPoseSupplier;

  public Superstructure(
      Elevator elevator,
      EndEffector endEffector,
      Arm arm,
      GroundIntakeRollers intakeRollers,
      IntakePivot intakePivot,
      Supplier<Pose2d> robotPoseSupplier) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.arm = arm;
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
    this.robotPoseSupplier = robotPoseSupplier;

    this.reefOnRight =
        new Trigger(
            () ->
                !isStructureOnLeft(
                    robotPoseSupplier.get(),
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Blue
                        ? FieldConstants.Reef.center
                        : ChoreoAllianceFlipUtil.flip(FieldConstants.Reef.center)));

    this.bargeOnRight =
        new Trigger(
            () ->
                !isStructureOnLeft(
                    robotPoseSupplier.get(),
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Blue
                        ? FieldConstants.Barge.middleCage
                        : ChoreoAllianceFlipUtil.flip(FieldConstants.Barge.middleCage)));

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    for (StructureState state : StructureState.values()) {
      prevStateTriggers.put(state, new Trigger(() -> this.prevState == state));
    }

    configStateTransitions();
  }

  public void configStateTransitions() {
    stateTriggers.get(StructureState.IDLE);
    // Move elevator and reef to L1, no safety limits since arm is still safe
    stateTriggers
        .get(StructureState.L1)
            .onTrue(intakePivot.goToL1());

    // L2 and L3 are same arm position so they are put together, once again no safety limits
    stateTriggers.get(StructureState.L2).onTrue(elevator.toReefLevel(1));
    stateTriggers.get(StructureState.L3).onTrue(elevator.toReefLevel(2));
    stateTriggers
        .get(StructureState.L2)
        .or(stateTriggers.get(StructureState.L3))
        .onTrue(arm.toReefLevel(1, reefOnRight));

    // L4 reef level, no safety limits
    stateTriggers
        .get(StructureState.L4)
        .onTrue(elevator.toReefLevel(3))
        .and(elevator.reachedPosition)
        .debounce(.025)
        .onTrue(arm.toReefLevel(2, reefOnRight));

    // Scoring coral, depending on previous state it changes endEffector velocity
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L1))
            .onTrue(intakeRollers.outtakeL1());
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L2))
        .and(elevator.reachedPosition)
        .onTrue(endEffector.off())
        .onTrue(arm.toScoringPosition(1, reefOnRight));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L3))
        .and(elevator.reachedPosition)
        .onTrue(endEffector.off())
        .onTrue(arm.toScoringPosition(1, reefOnRight));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L4))
        .and(elevator.reachedPosition)
        .onTrue(endEffector.off())
        .onTrue(arm.toScoringPosition(2, reefOnRight));

    //    stateTriggers
    //        .get(StructureState.SCORE_CORAL)
    //        .and(arm.reachedPosition)
    //        .debounce(.025)
    //        .onTrue(endEffector.setCoralOuttakeVoltage())
    //        .debounce(.1)
    //        .onTrue(this.setState(StructureState.PREHOME));

    // Dealgae Levels
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .onTrue(elevator.toDealgaeLevel(0))
        .onTrue(arm.toDealgaeLevel(0, reefOnRight));

    stateTriggers
        .get(StructureState.DEALGAE_L3)
        .onTrue(elevator.toDealgaeLevel(1))
        .onTrue(arm.toDealgaeLevel(1, reefOnRight));
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .or(stateTriggers.get(StructureState.DEALGAE_L3))
        .onTrue(endEffector.setAlgaeIntakeVoltage());

    // Barge level
    stateTriggers
        .get(StructureState.BARGE)
        .onTrue(elevator.toBargePosition())
        .onTrue(arm.toPreBargeLevel(bargeOnRight));
    stateTriggers
        .get(StructureState.SCORE_ALGAE)
        .and(prevStateTriggers.get(StructureState.BARGE))
        .onTrue(arm.toBargeLevel(bargeOnRight))
        .debounce(0.2)
        .onTrue(endEffector.setAlgaeOuttakeVoltage());

    // Processor state
    stateTriggers
        .get(StructureState.PROCESSOR)
        .onTrue(elevator.toProcessorPosition())
        .and(elevator.reachedPosition)
        .debounce(.025) // wait 2 loop times
        .onTrue(arm.toProcessorLevel());

    // Outtake
    // stateTriggers.get(StructureState.SCORE_ALGAE).onTrue(endEffector.setAlgaeOuttakeVoltage());

    // Turn coral motor off (helpful for transitioning from SCORE_CORAL), do not turn algae motor
    // off since you might be holding one

    stateTriggers
        .get(StructureState.PREHOME)
        .and(
            prevStateTriggers
                .get(StructureState.HANDOFF)
                .or(prevStateTriggers.get(StructureState.PRE_HANDOFF))
                .negate())
        .onTrue(arm.toHome())
        .and(arm.isAtHome)
        .onTrue(this.setState(StructureState.HOME));

    stateTriggers
        .get(StructureState.PREHOME)
        .and(
            prevStateTriggers
                .get(StructureState.HANDOFF)
                .or(prevStateTriggers.get(StructureState.PRE_HANDOFF)))
        .onTrue(elevator.toPreHandoffHome())
        .debounce(.05)
        .onTrue(arm.toHome(reefOnRight))
        .and(arm.isSafePosition)
        .onTrue(this.setState(StructureState.HOME));

    // Once arm is safe, the elevator can also home, once everything is done we can go to the IDLE
    // state.
    // As a safety feature, the HOME state is only valid if the previous state was PREHOME ensuring
    // that you don't skip steps.
    stateTriggers
        .get(StructureState.HOME)
        .and(prevStateTriggers.get(StructureState.PREHOME))
        .onTrue(elevator.toHome())
        .onTrue(arm.toHome())
        .and(arm.reachedPosition)
        .onTrue(intakePivot.goToStow())
        .and(elevator.reachedPosition)
        .onTrue(this.setState(StructureState.IDLE));

    stateTriggers
        .get(StructureState.IDLE)
        .and(endEffector.gamePieceIntaken.negate())
        .onTrue(endEffector.off());

    // Kills all subsystems
    stateTriggers
        .get(StructureState.CANCEL_ALL)
        .onTrue(elevator.off())
        .onTrue(arm.off())
        .onTrue(endEffector.off());

    stateTriggers
        .get(StructureState.GROUND_INTAKE)
        .onTrue(intakePivot.goToGroundIntake())
        .onTrue(intakeRollers.intakeCoral())
        .and(intakeRollers.coralIntakeIn)
        .onTrue(this.setState(StructureState.PREHOME));

    stateTriggers
        .get(StructureState.PRE_HANDOFF)
        .onTrue(intakePivot.goToHandoff())
        .onTrue(elevator.toPreHandoffHome())
        .and(elevator.isSafeForArm)
        .and(intakePivot.reachedPosition)
        .debounce(.025)
        .onTrue(arm.toHandoffPosition(reefOnRight.negate()))
        .and(elevator.reachedPosition)
        .and(arm.reachedPosition)
        .debounce(0.025)
        .onTrue(this.setState(StructureState.HANDOFF));

    stateTriggers
        .get(StructureState.HANDOFF)
        .onTrue(elevator.toHandoffPosition())
        .and(elevator.reachedPosition)
        .debounce(.03)
        .onTrue(intakeRollers.handoffCoral())
        .onTrue(endEffector.intakeCoral())
        .and(intakeRollers.coralIntakeIn.negate())
        .and(endEffector.gamePieceIntaken)
        .onTrue(endEffector.setEEVoltage(() -> -1))
        .onTrue(this.setState(StructureState.PREHOME));
    ;
  }

  public static boolean isStructureOnLeft(Pose2d robotPose, Translation2d reefPosition) {
    // Vector from robot to reef
    double dx = reefPosition.getX() - robotPose.getX();
    double dy = reefPosition.getY() - robotPose.getY();

    double theta = robotPose.getRotation().getRadians();

    double cross = Math.sin(theta) * dx - Math.cos(theta) * dy;

    return cross < 0;
  }

  // call manually
  public void periodic() {
    Logger.recordOutput(
        "Superstructure/ManipulatorSide", this.manipulatorSide.toString()); // TODO: remove
    Logger.recordOutput("Superstructure/State", this.state.toString());
    Logger.recordOutput("Superstructure/PrevState", this.prevState.toString());
    Logger.recordOutput("Superstructure/StateTime", this.stateTimer.get());

    LoggedTracer.record(this.getClass().getSimpleName());
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state == state ? this.prevState : this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }

  public StructureState getState() {
    return this.state;
  }

  public StructureState getPrevState() {
    return this.prevState;
  }

  public Command setManipulatorSide(ManipulatorSide side) {
    return Commands.runOnce(
        () -> {
          this.manipulatorSide = side;
          // set manipulator side
        });
  }
}
