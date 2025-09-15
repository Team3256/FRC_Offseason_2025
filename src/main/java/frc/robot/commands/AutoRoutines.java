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

    private static class AutoCommands {

        private final Elevator m_elevator;
        private final Arm m_arm;
        private final EndEffector m_endEffector;

        public AutoCommands(Elevator elevator, Arm arm, EndEffector endEffector) {

            m_elevator = elevator;
            m_arm = arm;
            m_endEffector = endEffector;
        }


    }
}
