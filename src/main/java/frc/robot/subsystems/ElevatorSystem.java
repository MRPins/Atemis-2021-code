// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotCharacteristics;
import frc.robot.io.ElevatorEncoders;

public class ElevatorSystem extends SubsystemBase {

    private WPI_TalonFX elevateR;
    private WPI_TalonFX elevateL;
    private Solenoid solenoidL;

    private final SingleJointedArmSim armSim;
    private final Field2d field;
    private final ElevatorEncoders encoders;

    public ElevatorSystem() {
        elevateL = new WPI_TalonFX(16);
        elevateR = new WPI_TalonFX(15);

        solenoidL = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

        if (Robot.isSimulation()) {
            armSim = new SingleJointedArmSim(
                RobotCharacteristics.ELEVATOR_MOTORS,
                RobotCharacteristics.ELEVATOR_GEAR_RATIO,
                RobotCharacteristics.ELEVATOR_MOMENT_OF_INERTIA,
                RobotCharacteristics.ELEVATOR_ARM_LENGTH_M,
                RobotCharacteristics.ELEVATOR_MIN_ARM_ANGLE_RAN,
                RobotCharacteristics.ELEVATOR_MAX_ARM_ANGLE_RAN,
                RobotCharacteristics.ELEVATOR_ARM_MASS_KG,
                false);
        }
        else{
            armSim = null;
            elevateL.setInverted(true);
        }

        field = new Field2d();
        SmartDashboard.putData("field", field);

        encoders = new ElevatorEncoders(elevateR, elevateL, armSim);
        resetEncoders();
    }

    public void MoveUp() {
        elevateL.set(1);
        elevateR.set(1);
    }

    public void MoveDown() {
        elevateL.set(-1);
        elevateR.set(-1);
    }

    public void stop() {
        elevateL.set(0);
        elevateR.set(0);
    }

    public void toggle() {
        solenoidL.toggle();
    }

    public boolean getSolenoid() {
        return solenoidL.get();
    }

    public double getDistancePassedLeft() {
        return encoders.getDistancePassedLeft();
    }

    public double getDistancePassedRight() {
        return encoders.getDistancePassedRight();
    }

    public void resetEncoders() {
        encoders.resetEncoders();
    }

    @Override
    public void simulationPeriodic() {

        armSim.setInputVoltage(elevateL.get() * RobotController.getBatteryVoltage());

        armSim.update(0.02);
    }
}