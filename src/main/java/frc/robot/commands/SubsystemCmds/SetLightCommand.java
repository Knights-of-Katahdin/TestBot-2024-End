package frc.robot.commands.SubsystemCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class SetLightCommand extends Command {

    private final LEDSubsystem ledSubsystem;
    private final double power;

    public SetLightCommand(LEDSubsystem ledSubsystem, double power) {
        addRequirements(ledSubsystem);
        this.ledSubsystem = ledSubsystem;
        this.power = power;
    }

    @Override
    public void initialize() {
        ledSubsystem.setPower(power);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
