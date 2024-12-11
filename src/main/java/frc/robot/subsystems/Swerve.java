package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// swerve.java değiştirildi.
public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // Hardcoded
    boolean isRed = true;

    public Timer timer;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(2, Constants.Swerve.Mod0.constants),
            new SwerveModule(0, Constants.Swerve.Mod1.constants),
            new SwerveModule(3, Constants.Swerve.Mod2.constants),
            new SwerveModule(1, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            getGyroYaw(), 
            getModulePositions(), 
            new Pose2d()
        );

        AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0),
                        new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
                        Constants.Swerve.maxSpeed,
                        Math.hypot(Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0),
                        new ReplanningConfig()),
                        isRed(), 
                        this
                    );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                              : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    // eklendi, bu şekilde gyrodan her an veri çekebiliyoruz.
    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void resetGyro() {
        gyro.setYaw(0); // Gyro yönünü sıfırlar
    }    

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }



    // AUTONOMOUS //

    public Command followPathCommand(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return AutoBuilder.followPath(path);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(new Translation2d(speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond), 
            speeds.omegaRadiansPerSecond, 
            false, 
            false
        );
    }

    public BooleanSupplier isRed() {
        return () -> isRed;
    }

    public void setInitialPose(String pathName) {

        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if (isRed().getAsBoolean()) {
            resetPose(GeometryUtil.flipFieldPose(path.getPreviewStartingHolonomicPose()));
        }
        else {
            resetPose(path.getPreviewStartingHolonomicPose());
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        poseEstimator.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("yaw", getGyroYaw().getDegrees());
        SmartDashboard.putString("Pose", getPose().toString());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}
