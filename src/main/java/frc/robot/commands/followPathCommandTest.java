package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class followPathCommandTest extends Command {
    private final PathPlannerPath path;
    private final Swerve s_Swerve;
    private Timer timer = new Timer();
    private Pose2d initialPathPlannerPose = null;

    public followPathCommandTest(
            String pathName,
            Swerve s_Swerve,
            TrajectoryConfig config,
            boolean shouldMirrorPath,
            boolean isInitial
    ) {
        this.s_Swerve = s_Swerve;

        // Load path from file
        try {
            this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
            throw new RuntimeException(e);
        }
        
        if(isInitial){
            initialPathPlannerPose = path.getPreviewStartingHolonomicPose();
        }
        // Declare subsystem dependencies
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        if (initialPathPlannerPose != null){
            s_Swerve.resetOdometry(initialPathPlannerPose);
        }
    }

    @Override
    public void execute() {
        timer.reset();
        timer.start();

    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
