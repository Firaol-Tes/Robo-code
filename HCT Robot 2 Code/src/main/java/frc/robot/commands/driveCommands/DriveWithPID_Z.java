package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithPID_Z extends CommandBase
{
    //Bring in the Drive Train subsystem
    private static final DriveTrain drive = RobotContainer.driveTrain;
    private double setpointYaw; 

    //Create two PID Controllers
    PIDController pidZAxis;

    public DriveWithPID_Z(double setpointYaw, double epsilonYaw)
    {
        this.setpointYaw = setpointYaw;
        addRequirements(drive);

        pidZAxis = new PIDController(0.1, 0, 0);
        pidZAxis.setTolerance(epsilonYaw);
    }

    @Override
    public void initialize()
    {
        drive.resetEncoders();
        drive.resetYaw();
        pidZAxis.reset();
    }

    @Override
    public void execute()
    {
        drive.holonomicDrive(0.0,0.0,
         MathUtil.clamp(pidZAxis.calculate(drive.getYaw(), setpointYaw), -1, 1));
    }

    @Override
    public void end (boolean interrupted)
    {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished()
    {
        return pidZAxis.atSetpoint();
    }
}