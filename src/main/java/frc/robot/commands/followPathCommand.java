package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class followPathCommand extends Command {
    private final PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private final String pathName;
    private final Swerve s_Swerve;
    private final Rotation2d startingRotation;
    private final ChassisSpeeds startingSpeeds;
    private final boolean resetPoseOnStart;
    private Timer timer = new Timer();

    public followPathCommand(String pathName, Swerve s_Swerve, boolean isReversed, boolean resetPoseOnStart) {
        this.s_Swerve = s_Swerve;
        this.pathName = pathName;
        this.resetPoseOnStart = resetPoseOnStart;
        
        try {
            this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load path: " + e.getMessage(), e);
        }

        this.startingSpeeds = new ChassisSpeeds(0, 0, 0);
        this.startingRotation = new Rotation2d(0);

        try {
            this.trajectory = path.getTrajectory(startingSpeeds, startingRotation);
        } catch (Exception e) {
            throw new RuntimeException("Failed to get trajectory: " + e.getMessage(), e);
        }

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        if (resetPoseOnStart) {
           s_Swerve.setInitialPose(pathName);
        }

        if (trajectory == null) {
            throw new IllegalStateException("Trajectory is null! Check if the path is loaded correctly.");
        }

        s_Swerve.drive(new Translation2d(0, 0), 0, true, false);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();

        if (elapsedTime > trajectory.getTotalTimeSeconds()) {
            return;
        }

        PathPlannerTrajectory.State targetState = trajectory.sample(elapsedTime);

        // // Katsayıları uygula
        // double adjustedVelocity = targetState.velocityMps * Constants.AutoConstants.SPEED_SCALER;
        // // double adjustedAngularVelocity = targetState.headingAngularVelocityRps * Constants.PathFollower.ANGULAR_SCALING_FACTOR;

        // // Maksimum hız limitine uyum sağla
        // adjustedVelocity = Math.min(adjustedVelocity, Constants.Swerve.maxSpeed); 
        // // adjustedAngularVelocity = Math.min(adjustedAngularVelocity, Constants.Swerve.maxAngularVelocity);

        // System.out.println("Adjust Velocity: " + adjustedVelocity);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            targetState.velocityMps * Math.cos(targetState.heading.getRadians()),
            targetState.velocityMps * Math.sin(targetState.heading.getRadians()),
            targetState.headingAngularVelocityRps
        );

        s_Swerve.drive(
            new Translation2d(chassisSpeeds.vyMetersPerSecond , -chassisSpeeds.vxMetersPerSecond),
            -chassisSpeeds.omegaRadiansPerSecond,
            true, 
            false
        );
    }


    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, true, false);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
