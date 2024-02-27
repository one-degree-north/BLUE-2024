package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    
    public static final class StickConstants {
        public static final double stickDeadband = 0.1;    
        public static final int stickID = 1; 
    }

    public static final class IntaterConstants {
        //CAN ID
        public static final int flywheelLeftID = 0;
        public static final int flywheelRightID = 0;
        public static final int intakeID = 0;
        public static final int intakeSensorID = 0;

        //PID VALUES
        public static final double FlywheelLeftkP = 0;
        public static final double FlywheelLeftkI = 0;
        public static final double FlywheelLeftkD = 0;
        public static final double FlywheelLeftkFF = 0;
        public static final double FlywheelLeftkMaxOutput = 0;
        public static final double FlywheelLeftkMinOutput = 0;

        public static final double FlywheelRightkP = 0;
        public static final double FlywheelRightkI = 0;
        public static final double FlywheelRightkD = 0;
        public static final double FlywheelRightkFF = 0;
        public static final double FlywheelRightkMaxOutput = 0;
        public static final double FlywheelRightkMinOutput = 0;

        public static final double IntakekP = 0;
        public static final double IntakekI = 0;
        public static final double IntakekD = 0;
        public static final double IntakekFF = 0;
        public static final double IntakekMaxOutput = 0;
        public static final double IntakekMinOutput = 0;

        //POSITIOSN
        public static final double SpeakerVel = 0;
        public static final double AmpVel = 0;
        public static final double OuttakeVel = 0;
        public static final double IntakeVel = 0;
    }

    public static final class PivotConstants {
        //CAN ID
        public static final int pivotLeadID = 0;
        public static final int pivotFollowID = 0;

        //PID VALUES
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 0;
        public static final double kMinOutput = 0;

        //CONVERSION FACTOR (GEAR RATIO)
        public static final double PosConversionFactor = 0;
        public static final double VelConversionFactor = 0;

        //POSITIONS
        public static final double SpeakerPosition = 0;
        public static final double AmpPosition = 0;
        public static final double GroundIntakePosition = 0;
        public static final double SourceIntakePosition = 0;
    }
    
    public static final class ClimbConstants {
        //CAN ID
        public static final int climbMotorLeftID = 0;
        public static final int climbMotorRightID = 0;
        public static final int climbLeftEncoderID = 0;
        public static final int climbRightEncoderID = 0;

        //PID VALUES
        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kS = 0;
        public static final double kG = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        //POSITIONS
        public static final double ClimbUpPosition = 0;
        public static final double ClimbDownPosition = 0;

        public static final double DutyCycle = 0;

        //CLIMB OFFSET
        public static final double climbOffset = 0;
    }

    public static final class SwerveConstants {

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(0); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(0); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = Units.inchesToMeters(4.0)*Math.PI;


        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = (6.55 / 1.0);
        public static final double angleGearRatio = (72.0 / 14.0) * (24.0 / 12.0);

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive; //inverted true
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive; //inverted false

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive; //inverted false

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
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
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.2;
        public static final double angleKI = 0;
        public static final double angleKD = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
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
}
