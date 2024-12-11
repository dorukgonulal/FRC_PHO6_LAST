package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class test extends Command{

    private final Swerve s_Swerve = new Swerve();

    public Command followPathCommand(String pathName) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        ReplanningConfig replanningConfig = new ReplanningConfig();
            replanningConfig.enableInitialReplanning = false;
            replanningConfig.enableDynamicReplanning = false;

        return new FollowPathCommand(
                path,
                s_Swerve::getPose, // Robot pose supplier
                s_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                s_Swerve::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants, 
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
                        Constants.AutoConstants.driveBaseRadius
                ),
                new ReplanningConfig(), // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                s_Swerve // Reference to this subsystem to set requirements
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}
