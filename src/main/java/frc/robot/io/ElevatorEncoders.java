// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorEncoders extends SubsystemBase {

  private final WPI_TalonFX elevateR;
  private final WPI_TalonFX elevateL;

  private final SingleJointedArmSim armSim;

  private double robotZeroPosition;
  
  public ElevatorEncoders(WPI_TalonFX elevateR, WPI_TalonFX elevateL, SingleJointedArmSim armSim) {
    this.elevateL = elevateL;
    this.elevateR = elevateR;
    this.armSim = armSim;
    this.robotZeroPosition = 0;
  }

  
  public double getDistancePassedLeft() {
    if (Robot.isReal()) {
        return Math.toRadians(elevateL.getSelectedSensorPosition() / Constants.TALON_FX_PPR * 360);
    } else {
        return armSim.getAngleRads() - robotZeroPosition;
    }
}

public double getDistancePassedRight() {
    if (Robot.isReal()) {
        return elevateR.getSelectedSensorPosition() / Constants.TALON_FX_PPR * 360;
    } else {
        return armSim.getAngleRads() - robotZeroPosition;
    }
}

public void resetEncoders() {
    if (Robot.isReal()) {
        elevateL.setSelectedSensorPosition(0);
        elevateR.setSelectedSensorPosition(0);
    } else {
        this.robotZeroPosition = armSim.getAngleRads();
        this.robotZeroPosition = armSim.getAngleRads();
    }
}

}
