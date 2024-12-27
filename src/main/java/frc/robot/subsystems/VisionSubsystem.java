package frc.robot.subsystems;




import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable limelightTable;

    private boolean isConnected;
    private long lastUpdatedTime;
    private long lastTimeChecked;


  


    public VisionSubsystem() {


        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        SmartDashboard.putBoolean("Valid Target", hasValidTarget());

        intializeLimelight(1);

        
        isConnected = true;
    }

    public void intializeLimelight(int pipeline) {

        //setLedMode(3);
        setCamMode(0);
        setUltraPipeline(pipeline);
        //setPiPMode(2);

    }

    /**
     * Wether the Limelight has a valid target
     * 
     * @return A boolean that is true when the Limelight has a valid target.
     */
    public boolean hasValidTarget() {

        return limelightTable.getEntry("tv").getDouble(0) == 1;

    }

    /**
     * Gets horizontal offset of the Limelight's crosshair
     * 
     * @return The horizontal offset of the Limelight's crosshair.
     */
    public double getTargetHorizontalOffset() {

        return limelightTable.getEntry("tx").getDouble(0.0);

    }

    /**
     * Gets vertical offset of the Limelight's crosshair
     * 
     * @return The vertical offset of the Limelight's crosshair.
     */
    public double getTargetVerticalOffset() {

        return limelightTable.getEntry("ty").getDouble(0.0);

    }

    /**
     * Gets the targets area in relation the the image size
     * 
     * @return The area of the target in relationship to the image size.
     */
    public double getTargetArea() {

        return limelightTable.getEntry("ta").getDouble(0.0);

    }

    /**
     * Gets distance from the front of the shooter to the target.
     * 
     * @return The distance from the shooter to the LimeLight target.
     */
    public double getTargetDistance() {

        if (!hasValidTarget())
            return 0.0;

        double angleToGoalDegrees = Constants.LimelightConstants.MOUNTED_ANGLE + getTargetVerticalOffset();
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return ((Constants.LimelightConstants.TARGET_HEIGHT - Constants.LimelightConstants.LENS_HEIGHT)
                / Math.tan(angleToGoalRadians));

    }

    /**
     * Gets the current camera mode of the Limelight.
     * 
     * @param defaultValue The default value if the NetworkTableEntry could not be
     *                     found.
     * @return The current camera mode of the Limelight.
     */
    public double getCamMode(double defaultValue) {

        return limelightTable.getEntry("camMode").getDouble(defaultValue);
    
    }

    /**
     * Gets the current LED mode of the Limelight.
     * 
     * @param defaultValue The default value if the NetworkTableEntry could not be
     *                     found.
     * @return The current LED mode of the Limelight.
     */
    public double getLedMode(double defaultValue) {

        return limelightTable.getEntry("ledMode").getDouble(defaultValue);

    }

    public boolean getConnected() {

        return isConnected;
    
    }

    /**
     * Sets the camera mode of the Limelight to a given camera mode.
     * 
     * @param camMode The given camera mode to set the LimeLight to.
     */
    public void setCamMode(double camMode) {

        limelightTable.getEntry("camMode").setNumber(camMode);
    
    }

    /**
     * Set the LED mode of the Limelight to a given LED mode.
     * 
     * @param ledMode The given LED mode to set the Limelight to.
     */
    public void setLedMode(double ledMode) {
        limelightTable.getEntry("ledMode").setNumber(ledMode);
    }

    public void setPiPMode(double pipMode) {
        limelightTable.getEntry("stream").setNumber(pipMode);
    }

    public void setPipeline(double pipeline){

        limelightTable.getEntry("pipeline").setNumber(pipeline);

    }

    public void setUltraPipeline(int pipeline) {

		NetworkTableEntry pipelineEntry = limelightTable.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);

    }

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {

        return getLimelightNTTable(tableName).getEntry(entryName);

    }




    public static void setLimelightNTDouble(String tableName, String entryName, double val) {

        getLimelightNTTableEntry(tableName, entryName).setDouble(val);

    }

    public static void setCameraMode_Processor(String limelightName) {

        setLimelightNTDouble(limelightName, "camMode", 0);

    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {

        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);

    }

    public static void setCameraMode_Driver(String limelightName) {

        setLimelightNTDouble(limelightName, "camMode", 1);

    }

    @Override
    public void periodic() {

        isConnected = true;

        long anyLastUpdated = limelightTable.getEntry("ta").getLastChange();
        long current = System.nanoTime();

        if (anyLastUpdated == lastUpdatedTime) {
            if (current - lastTimeChecked > 1_000_000_000) {
                isConnected = false;
            }
        }

        else {
            lastUpdatedTime = anyLastUpdated;
            lastTimeChecked = current;
        }

        SmartDashboard.putNumber("Target Distance", getTargetDistance());
        SmartDashboard.putBoolean("Valid Target", hasValidTarget());
        SmartDashboard.putNumber("Target Horizontal Ofset", getTargetHorizontalOffset());

        //System.out.println(getConnected() + " - " + anyLastUpdated + " - " + current);

    }
}

