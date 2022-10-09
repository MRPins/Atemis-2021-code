package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.Constants;
public class StickDrive extends CommandBase {

    DriveSystem driveSystem;
    PS4Controller ps4;
    Constants Constants;

    /** Creates a new NewDriveCommand. */
    public StickDrive(DriveSystem driveSystem, PS4Controller ps4) {
        this.driveSystem = driveSystem;
        this.ps4 = ps4;
        addRequirements(driveSystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double R2 = ps4.getR2Axis(); // ערכים של R2
        double LJX = ps4.getLeftX()*0.3; // ערכי האיקס של הג'ויסטיק השמאלי
        double L2 = ps4.getL2Axis();
        double speed = 0;
        if(R2 >= L2 + Constants.MIN_SPEED){
            speed = R2;
            driveSystem.drive(speed +LJX,speed - LJX);
        }
        else if(L2 >= R2 + Constants.MIN_SPEED){
            speed = L2;
            driveSystem.drive(-speed +LJX,-speed - LJX);
        }
        else if(R2 + L2 == 0)
        driveSystem.drive(ps4.getLeftX(),-ps4.getLeftX());
        else
        driveSystem.Stop();
        
        
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSystem.Stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double fixAxis(double value) {
        return (value + 1) / 2.0;
    }
}