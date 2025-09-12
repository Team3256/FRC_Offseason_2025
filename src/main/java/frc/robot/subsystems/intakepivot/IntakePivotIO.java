package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
    @AutoLog
    public class IntakePivotIOInputs {
        public double leftMotorVoltage = 0.0;
        public double leftMotorVelocity = 0.0;
        public double leftMotorPosition = 0.0;
        public double leftMotorStatorCurrent = 0.0;
        public double leftMotorSupplyCurrent = 0.0;

        public double rightMotorVoltage = 0.0;
        public double rightMotorVelocity = 0.0;
        public double rightMotorPosition = 0.0;
        public double rightMotorStatorCurrent = 0.0;
        public double rightMotorSupplyCurrent = 0.0;

    }

    public default void updateInputs(IntakePivotIOInputs inputs) {}

    public default void setPosition(Angle position, AngularVelocity velocity) {}

    public default void setPosition(Angle position) {}

    public default void setPosition(double position) {}

    public default void setVoltage(Voltage voltage) {}

    public default TalonFX getMotor() {
        return new TalonFX(0);
    }

    public default void resetPosition(Angle angle) {}

    public default void off() {}
}
