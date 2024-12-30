package frc.robot.commands.SubsystemCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/** Default command for the LED subsystem.  Doesn't change the color, leaves it as is. */
public class KeepLightCommand extends Command {

    public KeepLightCommand(LEDSubsystem ledSubsystem) {
        addRequirements(ledSubsystem);
    }
    
}
