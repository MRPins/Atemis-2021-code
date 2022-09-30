// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorEncoders extends SubsystemBase {

  private final WPI_TalonFX elevateR;
  private final WPI_TalonFX elevateL;

  private final SingleJointedArmSim armSim;

  private double robotZeroPosition;
  private boolean pass180;
  
  public ElevatorEncoders(WPI_TalonFX elevateR, WPI_TalonFX elevateL, SingleJointedArmSim armSim) {
    this.elevateL = elevateL;
    this.elevateR = elevateR;
    this.armSim = armSim;
    pass180 = false;
  }

  
  public double getDistancePassedLeft() {
    double pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads())) - 1);
    if (Robot.isReal()) {
        return elevateL.getSelectedSensorPosition() / Constants.TALON_FX_PPR * 360;
    } else {
        
        if (pos > 2){
            pass180 = true;
        }

        if (pass180 && Math.cos(Math.toDegrees(armSim.getAngleRads())) < 0){
            pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads())) + 3);
        }
        else if (pass180 && Math.cos(Math.toDegrees(armSim.getAngleRads())) > 0){
            pos = Math.cos(Math.toDegrees(armSim.getAngleRads())) + 3;
        }
        else if (pos < 0){
            pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads()))) + 1;
        }
        
        return (pos - robotZeroPosition) / Constants.TALON_FX_PPR * 360;
    }
}

public double getDistancePassedRight() {

    double pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads())) - 1);
    
    if (Robot.isReal()) {
        return elevateR.getSelectedSensorPosition() / Constants.TALON_FX_PPR * 360;
    } else {
        
        if (pos > 1.9){
            pass180 = true;
        }

        if (pass180 && Math.cos(Math.toDegrees(armSim.getAngleRads())) < 0){
            pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads())) + 3);
        }
        else if (pass180 && Math.cos(Math.toDegrees(armSim.getAngleRads())) > 0){
            pos = Math.cos(Math.toDegrees(armSim.getAngleRads())) + 3;
        }
        else if (pos > 0){
            pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads())) - 1);
        }
        else if (pos < 0){
            pos = Math.abs(Math.cos(Math.toDegrees(armSim.getAngleRads()))) + 1;
        }

        return (pos - robotZeroPosition) / Constants.TALON_FX_PPR * 360;
    }
}

public void resetEncoders() {
    if (Robot.isReal()) {
        elevateL.setSelectedSensorPosition(0);
        elevateR.setSelectedSensorPosition(0);
    } else {
        robotZeroPosition = 0;
    }
}

}
