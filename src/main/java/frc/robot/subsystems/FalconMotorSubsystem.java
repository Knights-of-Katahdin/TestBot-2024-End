package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconMotorConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.playingwithfusion.TimeOfFlight;



public class FalconMotorSubsystem extends SubsystemBase {

 
    private double targetPosition;
    private double TEST_REDUCTION_RATIO = 2;

   // private DigitalInput limitSwitch = new DigitalInput(0);
   // private DutyCycleEncoder absEncoder = new DutyCycleEncoder(1);
   // private TimeOfFlight tofSensor = new TimeOfFlight(0);


    private final TalonFX m_fx = new TalonFX(FalconMotorConstants.MOTOR_PORT);
    private final TalonFXConfigurator talonFXConfigurator = m_fx.getConfigurator();
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);



    public FalconMotorSubsystem(){

    
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigurator.apply(motorConfigs);
       

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25 / TEST_REDUCTION_RATIO; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12 / TEST_REDUCTION_RATIO; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01 / TEST_REDUCTION_RATIO; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 8 / TEST_REDUCTION_RATIO; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0 / TEST_REDUCTION_RATIO; // no output for integrated error
        slot0Configs.kD = 0.1 / TEST_REDUCTION_RATIO; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 95 / TEST_REDUCTION_RATIO; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 60 / TEST_REDUCTION_RATIO; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 600 / TEST_REDUCTION_RATIO; // Target jerk of 1600 rps/s/s (0.1 seconds)

   

        /* Retry config apply up to 5 times, report if failure */

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_fx.getConfigurator().apply(talonFXConfigs);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure device. Error: " + status.toString());
        }



        m_fx.setPosition(0);
        m_fx.setNeutralMode(NeutralModeValue.Brake);



      
    }

    @Override
    public void periodic() {
        //System.out.println("Target Position " + targetPosition);
        m_fx.setControl(m_request.withPosition(targetPosition * FalconMotorConstants.GEAR_RATIO / 360));
       // System.out.println("Time of Flight: "  + tofSensor.getRange());
       // System.out.println("Encoder Output: " + Units.radiansToDegrees(absEncoder.getAbsolutePosition()));
       // System.out.println("Limit Switch: " + limitSwitch.get());
  
    }
 
    public void setPosition(double targetPosition){
        this.targetPosition = targetPosition;
    }


    public double getTargetPosition(){
        return targetPosition;
    
    }

    public double getMeasuredPosition(){
        return m_fx.getPosition().getValueAsDouble()  / 2 / FalconMotorConstants.GEAR_RATIO * 360;
       
    
    }

    public void resetPosition(){
        m_fx.setPosition(0);
        targetPosition = 0;
    }

 
}
    



    



