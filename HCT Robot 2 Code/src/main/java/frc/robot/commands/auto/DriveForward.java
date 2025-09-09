package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveWithPID;
import frc.robot.commands.driveCommands.DriveWithPID_X;
import frc.robot.commands.driveCommands.ResetElevator;
import frc.robot.commands.driveCommands.SetClawPos;
import frc.robot.commands.driveCommands.SetElevator;

public class DriveForward extends AutoCommand
{
    public DriveForward ()
    {
        super(
            new DriveWithPID(1000, 10, 0, 0),
            new WaitCommand(0.5), 
            new DriveWithPID_X(500, 5, 0, 0),
            new WaitCommand(0.5), 
            new ResetElevator(), 
            new SetClawPos(180), 
            new SetElevator(-200, 5), 
            new SetClawPos(130), 
            new SetElevator(100, 5), 
            new DriveWithPID(-500, 10, 0, 0)
         
             
        
        );
            
    }
}