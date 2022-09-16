package frc.robot.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAxisTrigger extends Trigger {

    private final GenericHID hid;
    private final int axisIndex;
    private final double valueThreshold;

    public JoystickAxisTrigger(GenericHID hid, int axisIndex, double valueThreshold) {
        this.hid = hid;
        this.axisIndex = axisIndex;
        this.valueThreshold = valueThreshold;
    }

    @Override
    public boolean get() {
        double axisValue = hid.getRawAxis(axisIndex);
        return axisValue > valueThreshold;
    }
}
