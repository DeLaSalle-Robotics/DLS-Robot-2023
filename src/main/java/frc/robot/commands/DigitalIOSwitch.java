package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DigitalIOSwitch extends Button{
    DigitalInput digitalInput;

    public DigitalIOSwitch(int channel) {
        digitalInput = new DigitalInput(channel);
    }

    public boolean get() {
        return digitalInput.get();
    }
}
