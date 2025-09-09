package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithPID_X extends CommandBase
{
    //Bring in the Drive Train subsystem
    private static final DriveTrain drive = RobotContainer.driveTrain;

    private double setpointDistance;
    private double setpointYaw; 

    //Create two PID Controllers
    PIDController pidXAxis;
    PIDController pidZAxis;

    public DriveWithPID_X(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw)
    {
        this.setpointDistance = setpointDistance;
        this.setpointYaw = setpointYaw;
        addRequirements(drive);

        pidXAxis = new PIDController(1, 0, 0);
        pidXAxis.setTolerance(epsilonDistance);

        pidZAxis = new PIDController(0.1, 0, 0);
        pidZAxis.setTolerance(epsilonYaw);
    }

    @Override
    public void initialize()
    {
        drive.resetEncoders();
        drive.resetYaw();
        pidXAxis.reset();
        pidZAxis.reset();
    }

    @Override
    public void execute()
    {
        drive.holonomicDrive(
         MathUtil.clamp(pidXAxis.calculate(drive.getBackEncoderDistance(), setpointDistance), -0.5, 0.5),
         0.0,
         MathUtil.clamp(pidZAxis.calculate(drive.getYaw(), setpointYaw), -1, 1));
    }

    @Override
    public void end (boolean interrupted)
    {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);;
    }

    @Override
    public boolean isFinished()
    {
        return pidXAxis.atSetpoint();
    }
}