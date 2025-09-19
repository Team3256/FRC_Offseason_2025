// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.autoaim.CoralTargets;
import frc.robot.utils.autoaim.SourceIntakeTargets;

public class AutoRoutines {
    private final AutoFactory m_factory;

    private final AutoCommands m_autoCommands;

    private final Elevator m_elevator;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Arm m_arm;
    private final EndEffector m_endEffector;

    public AutoRoutines(
            AutoFactory factory,
            Elevator elevator,
            Arm arm,
            EndEffector endEffector,
            CommandSwerveDrivetrain drivetrain) {
        m_factory = factory;
        m_elevator = elevator;
        m_arm = arm;
        m_drivetrain = drivetrain;
        m_endEffector = endEffector;
        m_autoCommands = new AutoCommands(elevator, arm, endEffector);
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
        final AutoTrajectory preloadI = routine.trajectory("Left-I");

        routine.active().onTrue(preloadI.resetOdometry().andThen(Commands.waitSeconds(5)).andThen(preloadI.cmd()));
        preloadI.atTimeBeforeEnd(0.5).onTrue(m_autoCommands.goToL4());
        preloadI.done().onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(0.1))
            .andThen(m_autoCommands.scoreL4().asProxy())
            .withTimeout(1.0)
            .deadlineFor(
            m_drivetrain.pidToPose(
                () -> preloadI.getFinalPose().orElse(CoralTargets.BLUE_I.location))
            )
            .andThen(m_autoCommands.home().asProxy()));
            

        return routine;
    }

    private static class AutoCommands {

        private final Elevator m_elevator;
        private final Arm m_arm;
        private final EndEffector m_endEffector;

        public AutoCommands(Elevator elevator, Arm arm, EndEffector endEffector) {

            m_elevator = elevator;
            m_arm = arm;
            m_endEffector = endEffector;
        }

        public Command goToL4() {
            return m_elevator.toReefLevel(3)
            .alongWith(Commands.waitUntil(new Trigger(() -> m_elevator.getPosition() > 4))
            .withTimeout(0.5)
            .andThen(m_arm.toReefLevel(2, () -> true)));
        }

        public Command scoreL4() {
            return m_endEffector.setCoralOuttakeVoltage();
        }

        public Command home() {
            return m_elevator.toHome().alongWith(m_endEffector.off()).alongWith(m_arm.toHome());
        }

    }
}
