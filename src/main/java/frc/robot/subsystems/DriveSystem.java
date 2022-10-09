// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotCharacteristics;
import frc.robot.io.DriveTrainEncoders;

public class DriveSystem extends SubsystemBase {

    private WPI_TalonFX talonLF;
    private WPI_TalonFX talonLR;
    private WPI_TalonFX talonRR;
    private WPI_TalonFX talonRF;

    private final DifferentialDrivetrainSim driveTrainSim;
    private final Field2d field;
    private final DriveTrainEncoders encoders;


    public DriveSystem() {
        talonLF = new WPI_TalonFX(2);
        talonLR = new WPI_TalonFX(1);
        talonRR = new WPI_TalonFX(3);
        talonRF = new WPI_TalonFX(4);

        if (Robot.isSimulation()) {
            driveTrainSim = new DifferentialDrivetrainSim(
                    RobotCharacteristics.DRIVE_MOTORS,
                    RobotCharacteristics.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO,
                    RobotCharacteristics.DRIVE_MOMENT_OF_INERTIA,
                    RobotCharacteristics.WEIGHT_KG,
                    RobotCharacteristics.DRIVE_WHEEL_RADIUS_M,
                    RobotCharacteristics.DRIVE_TRACK_WIDTH_M,
                    null
            );
        } else {
            driveTrainSim = null;

            talonLF.setInverted(false);
            talonLR.setInverted(false);
            talonRF.setInverted(true);
            talonRR.setInverted(true);
        }

        field = new Field2d();
        SmartDashboard.putData("field", field);

        encoders = new DriveTrainEncoders(talonLF, talonRF, driveTrainSim);
        resetEncoders();
    }

   

    public double getDistancePassedLeftM() {
        return encoders.getDistancePassedLeftM();
    }

    public double getDistancePassedRightM() {
        return encoders.getDistancePassedRightM();
    }

    public double getDistancePassedM() {
        return encoders.getDistancePassedM();
    }

    public void resetEncoders() {
        encoders.resetEncoders();
    }

    public void drive(double lSpeed, double rSpeed) {
       
            talonLR.set(lSpeed);
            talonLF.set(lSpeed);
            talonRR.set(rSpeed);
            talonRF.set(rSpeed);

    }

    public void Stop() {
        talonLR.set(0);
        talonLF.set(0);
        talonRR.set(0);
        talonRF.set(0);
    }

    @Override
    public void simulationPeriodic() {
        double rightVoltage = talonRF.get() * RobotController.getBatteryVoltage();
        double leftVoltage = talonLF.get() * RobotController.getBatteryVoltage();
        driveTrainSim.setInputs(leftVoltage, rightVoltage);

        driveTrainSim.update(0.02);

        field.setRobotPose(driveTrainSim.getPose());
    }
}
