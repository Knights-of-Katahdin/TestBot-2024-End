package frc.robot.com.swervedrivespecialties.swervelib;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface DriveController {
    TalonFX getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getStateDistance();
}
