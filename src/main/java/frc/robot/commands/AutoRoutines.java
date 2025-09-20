// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.autoaim.CoralTargets;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Superstructure m_superstructure;

  public AutoRoutines(
      AutoFactory factory, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    m_factory = factory;
    m_drivetrain = drivetrain;
    m_superstructure = superstructure;
  }

  public AutoRoutine mobilityLeft() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityLeft");
    final AutoTrajectory mobilityTop = routine.trajectory("MobilityTop");
    routine.active().onTrue(mobilityTop.resetOdometry().andThen(mobilityTop.cmd()));
    return routine;
  }

  public AutoRoutine mobilityRight() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityRight");
    final AutoTrajectory mobilityBottom = routine.trajectory("MobilityBottom");
    routine.active().onTrue(mobilityBottom.resetOdometry().andThen(mobilityBottom.cmd()));
    return routine;
  }

  public AutoRoutine l4PreloadI() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadI");
    final AutoTrajectory preloadI = routine.trajectory("LEFT-I");

    routine
        .active()
        .onTrue(preloadI.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(preloadI.cmd()));
    preloadI.atTimeBeforeEnd(0.5).onTrue(m_superstructure.setState(StructureState.L4));
    preloadI.done().onTrue(m_superstructure.setState(StructureState.SCORE_CORAL));
            m_drivetrain.pidToPose(
                () -> preloadI.getFinalPose().orElse(CoralTargets.BLUE_I.location));

    return routine;
  }

  public AutoRoutine l4PreloadH() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadH");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");
    routine
        .active()
        .onTrue(preloadH.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(preloadH.cmd()));
    preloadH.atTimeBeforeEnd(0.5).onTrue(m_superstructure.setState(StructureState.L4));
    preloadH.done().onTrue(m_superstructure.setState(StructureState.SCORE_CORAL));
            m_drivetrain.pidToPose(
                () -> preloadH.getFinalPose().orElse(CoralTargets.BLUE_H.location));
    return routine;
  }

  public AutoRoutine l4PreloadG() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadG");
    final AutoTrajectory preloadG = routine.trajectory("MID-G");
    routine
        .active()
        .onTrue(preloadG.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(preloadG.cmd()));
    preloadG.atTimeBeforeEnd(0.5).onTrue(m_superstructure.setState(StructureState.L4));
    preloadG.done().onTrue(m_superstructure.setState(StructureState.SCORE_CORAL));
            m_drivetrain.pidToPose(
                () -> preloadG.getFinalPose().orElse(CoralTargets.BLUE_G.location));
    return routine;
  }

  public AutoRoutine l4PreloadF() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadF");
    final AutoTrajectory preloadF = routine.trajectory("RIGHT-F");
    routine
        .active()
        .onTrue(preloadF.resetOdometry().andThen(Commands.waitSeconds(2)).andThen(preloadF.cmd()));
    preloadF.atTimeBeforeEnd(0.5).onTrue(m_superstructure.setState(StructureState.L4));
    preloadF.done().onTrue(m_superstructure.setState(StructureState.SCORE_CORAL));
    m_drivetrain.pidToPose(
        () -> preloadF.getFinalPose().orElse(CoralTargets.BLUE_F.location));
    return routine;
  }

}
