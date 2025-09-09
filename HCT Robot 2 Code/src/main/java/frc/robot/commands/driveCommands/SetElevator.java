package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OMS;

public class SetElevator extends CommandBase
{
    private static OMS oms = RobotContainer.oms;

   // private PIDController pidEleavtor;
    private Double setpointDistance;
    PIDController pidEleavtor;
    


    public SetElevator(double setpointDistance,double epsolonDistance)
    {
        this.setpointDistance = setpointDistance;
        addRequirements(oms);
        pidEleavtor = new PIDController(1.0, 0, 0);
        pidEleavtor.setTolerance(epsolonDistance);
        
    }

    @Override
    public void initialize()
    {
        oms.resetEncoders();
        pidEleavtor.reset();
    }

    @Override
    public void execute()
    {

        oms.setElevatorMotorSpeed(
            MathUtil.clamp(
                pidEleavtor.calculate(
                    oms.getElevatorEncoderDistance(), setpointDistance), -0.5, 0.5));
    }

    @Override
    public void end (boolean interrupted)
    {
        oms.setElevatorMotorSpeed(0.0);
    }

    @Override
    public boolean isFinished()
    {
        return pidEleavtor.atSetpoint();
    }
}

