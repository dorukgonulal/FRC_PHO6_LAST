package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(29); 
        public static final double wheelBase = Units.inchesToMeters(29); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 7.5; //steering ayarlama
        public static final double angleKI = 0.02;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.01; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; 
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.0; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; 
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //done
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9 ;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(74.2+90-18+180);//-113 //67  tamalandı
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        } 

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //done
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(72.2-90-7+90+180); //132.75 //-47.25 tamamlandı
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //done
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-27-65+85);//-39.25 //140.75
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

         
        /* Back Right Module - Module 3 */
        public static final class Mod3 { //done
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-3.6+74);//154.75 //-25.25
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Intake {
        
      public static final int TOP_ROLLER_ID = 25; // doğrulandı
      public static final int BOTTOM_ROLLER_ID = 20; // doğrulandı
      public static final int BACK_ROLLER_ID = 26; // doğrulandı
      
    }

    public static final class Pivot {
      public static final int PIVOT_MOTOR_ID = 52; // doğrulandı
    }

    public static final class Elevator {
      public static final int ELEVATOR_LEFT_MOTOR = 28; // doğrılandı
      public static final int ELEVATOR_RIGHT_MOTOR = 29; // doğrulandı
    }

    public static final class ElevatorConstants{

        public static final double ELEVATOR_KP = 0.05; //0.165
        public static final double ELEVATOR_KI = 0.0005;
        public static final double ELEVATOR_KD = 0.075;
    
        public static final double kEncoderTick2Meter = 0;
    
        public static final double ELEVATOR_POWER = 0.45; //.55
    
    
        public static final double ELEVATOR_SETPOINT_INTAKE = 0;
        public static final double ELEVATOR_SETPOINT_MIDDLE = 0;
        public static final float ELEVATOR_SETPOINT_SHOOTAMP = 62;
        public static final double ELEVATOR_TOLERANCE = 0;
       
      }
    
    
      public static final class LimelightConstants {
        
        public static final double MOUNTED_ANGLE = 250; //degrees
    
        public static final double TARGET_HEIGHT = Units.metersToInches(1.33); 
    
        public static final double LENS_HEIGHT = Units.metersToInches(1.35);
    
        public static final double TRACKED_TAG_ROATION_KP = 0; 
    
        public static final double DISTANCE_CONSTRAINT = 8.4; //5.5 -2.2
    
        public static final double TRAP_TARGET_DISTANCE = -0.17;
    
        }
    
      public static final class IntakeConstants {
        public static final double AMP_SHOOT_POWER = 0.2; // TODO: this might be tuned specific
        public static final double SPEAKER_SHOOT_POWER = 0.95; // TODO: this might be tuned specific
        public static final double TRAP_SHOOT_POWER = 0.77;
        public static final double INTAKE_ON_POWER = 0.3; // TODO: this might be tuned specific
        public static final double INTAKE_FEED_POWER = 0.2; // TODO: this might be tuned specific
      }
    
      public static final class PivotConstants {
        public static final double PIVOT_KP = 0.05;
        public static final double PIVOT_KI = 0.0002;
        public static final double PIVOT_KD = 0.0002;
    
    
        public static final double PIVOT_POWER = 0.45;
    
        public static final double AMP_SHOOT_SETPOINT = -15.6; // TODO: this might be tuned specific
        public static final double SPEAKER_SHOOT_SETPOINT = 0; // TODO: this might be tuned specific
        public static final double INTAKE_ON_SETPOINT = -25.2; // TODO: this might be tuned specific
        public static final double TRAP_SHOOT_SETPOINT = 0;
        public static final double INTAKE_CLOSE_SETPOINT = 0; // TODO: this might be tuned specific
        public static final double PODIUMNOTE_SETPOINT = -5.7; // TODO: this might be tuned specific
    
      }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IOConstants {
    
        public static final int k_DRIVER_CONTROLLER_PORT = 0;
        public static final int k_OPERATOR_CONTROLLER_PORT = 1;
        public static final double k_CONTROLLER_DEADBAND = 0.05;
    
      }
    
}
