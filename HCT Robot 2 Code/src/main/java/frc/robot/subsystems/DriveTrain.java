package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    /**
     * Motors
     */
    private TitanQuad leftMotor;
    private TitanQuad rightMotor;
    private TitanQuad backMotor;

    /**
     * Encoders
     */
    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private TitanQuadEncoder backEncoder;

    /**
     * Sensors
     */
    private AHRS navx;

    private AnalogInput RightIR;
    private AnalogInput LeftIR;

    private Ultrasonic Rsonar;
    private Ultrasonic Lsonar;

    /**
     * Shuffleboard
     */
    private ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");

    private NetworkTableEntry leftEncoderValue = tab.add("Left Encoder", 0).getEntry();
    private NetworkTableEntry rightEncoderValue = tab.add("Right Encoder", 0).getEntry();
    private NetworkTableEntry backEncoderValue = tab.add("Back Encoder", 0).getEntry();

    private NetworkTableEntry gyroValue = tab.add("NavX Yaw", 0).getEntry();

    private NetworkTableEntry LeftIRDistance = tab.add("Left IR Distance", 0).getEntry();
    private NetworkTableEntry RightIRDistance = tab.add("Right IR Distance", 0).getEntry();

    private NetworkTableEntry RsonarValue = tab.add("Right Sonar", 0).getEntry();
    private NetworkTableEntry LsonarValue = tab.add("Left Sonar", 0).getEntry();

    public DriveTrain() {
        /**
         * Motors
         */
        leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.M3);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.M0);
        backMotor = new TitanQuad(Constants.TITAN_ID, Constants.M1);

        /**
         * Encoders
         */
        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.M3, Constants.WHEEL_DIST_PER_TICK);
        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.M0, Constants.WHEEL_DIST_PER_TICK);
        backEncoder = new TitanQuadEncoder(backMotor, Constants.M1, Constants.WHEEL_DIST_PER_TICK);

        /**
         * Sensors
         */
        navx = new AHRS(SPI.Port.kMXP);

        RightIR = new AnalogInput(Constants.RightIR_CH);
        LeftIR = new AnalogInput(Constants.LeftIR_CH);

        // Rsonar = new Ultrasonic(RTrigger, REcho);
        // Lsonar = new Ultrasonic(LTrigger, LEcho);
        
        Rsonar = new Ultrasonic(8, 9);
        Lsonar = new Ultrasonic(10, 11);

    }

    public double getRightIR_Distance() {
        return (Math.pow(RightIR.getAverageVoltage(), -1.2045)) * 27.726;
    }

    public double getLeftIR_Distance() {
        return (Math.pow(LeftIR.getAverageVoltage(), -1.2045)) * 27.726;
    }

    public double getRsonar_Distance() {
        Rsonar.ping();
        Timer.delay(0.01);
        return Rsonar.getRangeMM();
    }

    public double getLsonar_Distance() {
        Rsonar.ping();
        Timer.delay(0.01);
        return Lsonar.getRangeMM();
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setLeftMotorSpeed(double speed) {
        leftMotor.set(speed);
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setRightMotorSpeed(double speed) {
        rightMotor.set(speed);
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setBackMotorSpeed(double speed) {
        backMotor.set(speed);
    }

    /**
     * Sets the speed of the drive motors
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setDriveMotorSpeeds(double leftSpeed, double rightSpeed, double backSpeed) {
        leftMotor.set(leftSpeed);
        Timer.delay(0.003);
        rightMotor.set(rightSpeed);
        Timer.delay(0.003);
        backMotor.set(backSpeed);
        Timer.delay(0.003);
    }

    /**
     * Drives the robot using a holonomic algorithim
     * <p>
     * 
     * @param x axis value (-1 to 1)
     * @param y axis value (-1 to 1)
     * @param z axis value (-1 to 1)
     */
    public void holonomicDrive(double x, double y, double z) {
        double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double leftSpeed = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double backSpeed = (-2 * x / 3) + z;

        double max = Math.abs(rightSpeed);
        if (Math.abs(leftSpeed) > max)
            max = Math.abs(leftSpeed);
        if (Math.abs(backSpeed) > max)
            max = Math.abs(backSpeed);

        if (max > 1) {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }

        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }

    /**
     * Gets the encoder distance for the left drive motor
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getLeftEncoderDistance() {
        return leftEncoder.getEncoderDistance();
    }

    /**
     * Gets the encoder distance for the right drive motor
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getRightEncoderDistance() {
        return rightEncoder.getEncoderDistance() * -1;
    }

    /**
     * Gets the encoder distance for the back drive motor
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getBackEncoderDistance() {
        return backEncoder.getEncoderDistance();
    }

    /**
     * Gets the average forward or reverse encoder distance
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getAverageForwardEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
    }

    /**
     * Call for the current Yaw angle from the internal NavX
     * <p>
     * 
     * @return yaw angle in degrees range -180° to 180°
     */
    public double getYaw() {
        return navx.getYaw();
    }

      /**
     * Returns the total accumulated yaw angle (Z Axis, in degrees)
     * reported by the sensor.
     *<p>
     * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees.
     * This ensures that algorithms that wouldn't want to see a discontinuity 
     * in the gyro output as it sweeps past 0 on the second time around.
     *<p>
     * Note that the returned yaw value will be offset by a user-specified
     * offset value; this user-specified offset value is set by 
     * invoking the zeroYaw() method.
     *<p>
     * @return The current total accumulated yaw angle (Z axis) of the robot 
     * in degrees. This heading is based on integration of the returned rate 
     * from the Z-axis (yaw) gyro.
     */
    
    /**
     * Resets all encoders
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }

    /**
     * Sets the Yaw value back to zero
     */
    public void resetYaw() {
        navx.zeroYaw();
    }

    /**
     * Periodic loop
     * <p>
     * Code in here will run every system loop (20 ms), this is ideal for putting
     * sensor data on the shuffleboard.
     */
    @Override
    public void periodic() {
        leftEncoderValue.setDouble(getLeftEncoderDistance());
        rightEncoderValue.setDouble(getRightEncoderDistance());
        backEncoderValue.setDouble(getBackEncoderDistance());

        gyroValue.setDouble(getYaw());

        LeftIRDistance.setDouble(getLeftIR_Distance());
        RightIRDistance.setDouble(getRightIR_Distance());

        RsonarValue.setDouble(getRsonar_Distance());
        LsonarValue.setDouble(getLsonar_Distance());

    }
}