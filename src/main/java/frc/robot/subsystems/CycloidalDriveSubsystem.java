package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CycloidalDriveConstants;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;



public class CycloidalDriveSubsystem extends SubsystemBase {

    private double targetPosition, errPositionSync, maxPositionASync;
    private double P_SYNCH = 0.005;

 
    private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private final TalonFX m_fxL = new TalonFX(CycloidalDriveConstants.MOTOR_LEFT_PORT);
    private final TalonFXConfigurator talonFXConfiguratorfxL = m_fxL.getConfigurator();
    
    MotorOutputConfigs motorConfigsm_fxL = new MotorOutputConfigs();


    private final TalonFX m_fxR = new TalonFX(CycloidalDriveConstants.MOTOR_RIGHT_PORT);
    private final TalonFXConfigurator talonFXConfiguratorfxR = m_fxR.getConfigurator();
    TalonFXConfiguration talonFXConfigsm_m_fxR = new TalonFXConfiguration();
    MotorOutputConfigs motorConfigsm_fxR = new MotorOutputConfigs();

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);


    public CycloidalDriveSubsystem(){

        // in init function

        motorConfigsm_fxL.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfiguratorfxL.apply(motorConfigsm_fxL);
        motorConfigsm_fxR.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfiguratorfxR.apply(motorConfigsm_fxR);        
        

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 95; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 60; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 600; // Target jerk of 1600 rps/s/s (0.1 seconds)

   

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status1 = StatusCode.StatusCodeNotInitialized;
        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status1 = m_fxL.getConfigurator().apply(talonFXConfigs);
        status2 = m_fxR.getConfigurator().apply(talonFXConfigs);
        if (status1.isOK() && status2.isOK()) break;
        }
        if (!(status1.isOK() && status2.isOK())) {
        System.out.println("Could not apply configs, error code: " + status1.toString() + " or " + status2.toString());
        }




        m_fxL.setPosition(0);
        m_fxL.setNeutralMode(NeutralModeValue.Brake);


        m_fxR.setPosition(0);
        m_fxR.setNeutralMode(NeutralModeValue.Brake);
;
        maxPositionASync = (m_fxR.getPosition().getValueAsDouble() -  m_fxL.getPosition().getValueAsDouble()) / CycloidalDriveConstants.GEAR_RATIO * 360;


      
    }

    @Override
    public void periodic() {
        

        m_fxL.setControl(m_request.withPosition(targetPosition * CycloidalDriveConstants.GEAR_RATIO /360).withFeedForward(-errPositionSync));
        m_fxR.setControl(m_request.withPosition(targetPosition * CycloidalDriveConstants.GEAR_RATIO /360).withFeedForward(errPositionSync));
        double currPositionASync =  (m_fxR.getPosition().getValueAsDouble() -  m_fxL.getPosition().getValueAsDouble()) / CycloidalDriveConstants.GEAR_RATIO * 360;
        errPositionSync = P_SYNCH * currPositionASync;

        if(Math.abs(currPositionASync) > Math.abs(maxPositionASync))
            maxPositionASync = currPositionASync;


    //    System.out.println("motor sensor position left " + m_fxL.getPosition().getValue()*360);
    //    System.out.println("motor sensor position right " + m_fxR.getPosition().getValue()*360);
    //    System.out.println("Max Position ASynch " + maxPositionASync);
    //    System.out.println("target position " + getTargetPosition());
    }

    public void setPosition(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public double getAvgMeasuredPosition(){
        return (m_fxL.getPosition().getValueAsDouble() + m_fxR.getPosition().getValueAsDouble()) / 2 / CycloidalDriveConstants.GEAR_RATIO * 360;
       
    
    }



    public double getTargetPosition(){
        return targetPosition;
    
    }

    public void resetPosition(){
        m_fxL.setPosition(0);
        m_fxR.setPosition(0);
        targetPosition = 0;
    }

 
}
    



    



