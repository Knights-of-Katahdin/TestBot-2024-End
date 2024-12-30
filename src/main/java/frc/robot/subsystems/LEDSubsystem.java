package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * This subsytem controls a REV Robotics BLINKIN controller and a 5V RGB ADDRESSABLELED strip.
 * 
 */
public class LEDSubsystem extends SubsystemBase {
    
    private Spark blinkin;
    private double power;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("networkTable");

    public LEDSubsystem() {
        blinkin = new Spark(LEDConstants.BLINKIN_PWM_PORT);
        setToTeamColor();
    }

    public void setToTeamColor() {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            power = LEDConstants.RED;
        } else {
            power = LEDConstants.BLUE;
        }
        NetworkTableEntry newColor = table.getEntry("LEDcolor");
        newColor.setDouble(power);
        blinkin.set(power);
    }

    @Override
    public void periodic() {
        // check networktables for color. any subystem or command might modify it.
        NetworkTableEntry newColor = table.getEntry("LEDcolor");
        power = newColor.getDouble(power); // default to current value of 'power' if networktable entry isn't a valid double
        blinkin.set(power);
    }

    public void setPower(double power) {
        NetworkTableEntry newColor = table.getEntry("LEDcolor");
        newColor.setDouble(power);
        this.power = power;
    }
    public double getPower() {
        return this.power;
    }
    

}
