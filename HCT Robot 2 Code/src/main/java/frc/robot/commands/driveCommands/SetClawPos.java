package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OMS;

public class SetClawPos extends CommandBase
{
    private static final OMS oms = RobotContainer.oms;

    private double degrees;


    public SetClawPos(double degrees)
    {
        this.degrees = degrees;
;
        addRequirements(oms);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        oms.setServoPosition(degrees);
    }

    @Override
    public void end (boolean interrupted)
    {
       
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}