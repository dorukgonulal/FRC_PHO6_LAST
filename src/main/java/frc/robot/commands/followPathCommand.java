package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
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
    private final boolean isInitial;
    private Timer timer = new Timer();

    public followPathCommand(String pathName, Swerve s_Swerve, boolean isReversed, boolean isInitial) {
        this.s_Swerve = s_Swerve;
        this.pathName = pathName;
        this.isInitial = isInitial;
        // Load path from file
        try {
            this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load path: " + e.getMessage(), e);
        }

        this.startingSpeeds = s_Swerve.getRobotRelativeSpeeds();
        this.startingRotation = s_Swerve.getHeading();

        try{
          this.trajectory = path.getTrajectory(startingSpeeds, startingRotation);
        } catch (Exception e) {
          throw new RuntimeException("Failed to get trajectory: " + e.getMessage(), e);
        }

        // Declare subsystem dependencies
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
    if (!isInitial){
      s_Swerve.setInitialPose(pathName);
    }

    timer.reset();
    timer.start();
    }

    @Override
    public void execute() {
            double elapsedTime = timer.get();

            // Sample the trajectory at the current time
            PathPlannerTrajectory.State targetState = trajectory.sample(elapsedTime);

            // Use velocity and angular velocity for control
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( //
                targetState.velocityMps * Math.cos(targetState.heading.getRadians()), //
                targetState.velocityMps * Math.sin(targetState.heading.getRadians()), //
                targetState.headingAngularVelocityRps, //
                s_Swerve.getHeading() //
            );

            // Command the drivetrain
            s_Swerve.drive(
              new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
              chassisSpeeds.omegaRadiansPerSecond,
              true,
              false
            );
}

    @Override
    public void end(boolean interrupted) {
      s_Swerve.drive(new Translation2d(0, 0), 0, false, false);
      timer.stop();
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
