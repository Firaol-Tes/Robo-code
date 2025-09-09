package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OMS;

public class ResetElevator extends CommandBase
{
    private static OMS oms = RobotContainer.oms;

    //private PIDController pidEleavtor;

    //private Double setpointDistance;


    public ResetElevator()
    {

        addRequirements(oms);
    }

    @Override
    public void initialize()
    {
        oms.resetEncoders();
    }

    @Override
    public void execute()
    {
        oms.setElevatorMotorSpeed(0.5);
    }

    @Override
    public void end (boolean interrupted)
    {
        oms.setElevatorMotorSpeed(0.0);
        oms.resetEncoders();
    }

    @Override
    public boolean isFinished()
    {
        if (oms.getHlimitSwitch() == 0.0){
            return true;
        }
        else{
            return false;
        }
    }
}