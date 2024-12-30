/* 
package frc.robot.com.swervedrivespecialties.swervelib.rev;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.com.swervedrivespecialties.swervelib.DriveController;
import frc.robot.com.swervedrivespecialties.swervelib.DriveControllerFactory;
import frc.robot.com.swervedrivespecialties.swervelib.MechanicalConfiguration;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;



public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, String _canbus, MechanicalConfiguration mechConfiguration) {
            
            
            SparkMax motor = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();

            
            config.signals.primaryEncoderPositionPeriodMs(20);
            config.smartCurrentLimit((int) currentLimit);
            config.voltageCompensation(nominalVoltage);
            config.idleMode(IdleMode.kBrake);


            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * mechConfiguration.getWheelDiameter() * mechConfiguration.getDriveReduction();
            config.encoder.positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(positionConversionFactor / 60.0);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final SparkMax motor;
        private final RelativeEncoder encoder;

        private ControllerImplementation(SparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public MotorController getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public double getStateDistance() {
            return encoder.getPosition();
        }
    }
}
*/