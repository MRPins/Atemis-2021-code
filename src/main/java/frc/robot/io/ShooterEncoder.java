package frc.robot.io;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterEncoder {

    private final TalonFX shootLMotor;
    private final FlywheelSim flywheelSim;

    public ShooterEncoder(TalonFX shootLMotor, FlywheelSim flywheelSim) {
        this.shootLMotor = shootLMotor;
        this.flywheelSim = flywheelSim;
    }

    public double getShooterRpm() {
        if (Robot.isReal()) {
            return shootLMotor.getSelectedSensorVelocity() / Constants.TALON_FX_PPR * 600 * Constants.SHOOTER_GEAR_RATIO;
        } else {
            return flywheelSim.getAngularVelocityRPM();
        }
    }
}
