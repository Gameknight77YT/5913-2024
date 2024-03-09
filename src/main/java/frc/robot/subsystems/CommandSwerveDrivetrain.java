package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Util.ModifiedSignalLogger;
import frc.robot.Util.SwerveVoltageRequest;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity);


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(8, 0, 0),//translation
                                            new PIDConstants(6.5, 0, 0),//rotation
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->DriverStation.getAlliance().get().equals(Alliance.Red), 
            this); // Subsystem for requirements

        

            
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
        
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /*private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    private SysIdRoutine m_driveSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
            null,
            this));

    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine m_steerSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
            null,
            this));

    private SysIdRoutine m_slipSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
            null,
            this));
    
    public Command runDriveQuasiTest(Direction direction)
    {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction)
    {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public Command runDriveSlipTest()
    {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }*/

    @Override
    public void periodic() {
        //SmartDashboard.putString("pose", this.getState().Pose.toString());
        Subsystem.super.periodic();
    }


    
}