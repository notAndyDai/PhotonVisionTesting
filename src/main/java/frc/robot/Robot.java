// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private AprilTagFieldLayout tagLayout;

  private Pose2d refPose;

  private RobotContainer m_robotContainer;

  private PhotonCamera cameraR;
  private PhotonCamera cameraL;

  private NetworkTableInstance mTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable mTable = mTableInstance.getTable("Vision");

  private StructPublisher<Pose3d> mEstLPosePub = mTable.getStructTopic("Est Left", Pose3d.struct).publish();
  private StructPublisher<Pose3d> mEstRPosePub = mTable.getStructTopic("Est Right", Pose3d.struct).publish();
  private StructPublisher<Pose2d> mEstPosePub = mTable.getStructTopic("Pose Estimate", Pose2d.struct).publish();
  private StructPublisher<Pose2d> mRefPosePub = mTable.getStructTopic("Reference Pose", Pose2d.struct).publish();
  private StructPublisher<Transform2d> mErrPosePub = mTable.getStructTopic("Pose Estimate Error", Transform2d.struct)
      .publish();

  private PhotonPoseEstimator poseEstL;
  private PhotonPoseEstimator poseEstR;

  private DifferentialDrivePoseEstimator poseEst;

  // Controllers
  private Joystick controller;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    try {
      tagLayout = new AprilTagFieldLayout("src\\main\\deploy\\homeMap.json");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    m_robotContainer = new RobotContainer();

    refPose = new Pose2d(new Translation2d(), new Rotation2d());

    cameraL = new PhotonCamera("theLeftOne");
    cameraR = new PhotonCamera("theRightOne");

    poseEstL = new PhotonPoseEstimator(
        tagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraL,
        new Transform3d(
            new Translation3d(0, 0.08224442918505, 0.5),
            new Rotation3d(Math.PI, Units.degreesToRadians(11.6), Units.degreesToRadians(-22))));

    poseEstR = new PhotonPoseEstimator(
        tagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraR,
        new Transform3d(
            new Translation3d(0, -0.08224442918505, 0.5),
            new Rotation3d(Math.PI, Units.degreesToRadians(11.6), Units.degreesToRadians(-22))));

    poseEst = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(5),
        new Rotation2d(),
        1,
        1,
        new Pose2d());

    // controller = new Joystick(0);

    mRefPosePub.set(refPose);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // if (controller.getRawButton(1)) {

    // }

    updatePoseEstimator();

    Pose2d estPose = poseEst.getEstimatedPosition();

    mEstPosePub.set(estPose);
    mErrPosePub.set(refPose.minus(estPose));

  }

  public void updatePoseEstimator() {
    Optional<EstimatedRobotPose> estL = getEstimatedGlobalPose(poseEstL, poseEst.getEstimatedPosition());

    if (estL.isPresent()) {
      EstimatedRobotPose est = estL.get();
      mEstLPosePub.set(est.estimatedPose);
      poseEst.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
    }

    Optional<EstimatedRobotPose> estR = getEstimatedGlobalPose(poseEstR, poseEst.getEstimatedPosition());

    if (estR.isPresent()) {
      EstimatedRobotPose est = estR.get();
      mEstRPosePub.set(est.estimatedPose);
      poseEst.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
    }

    poseEst.update(new Rotation2d(), new DifferentialDriveWheelPositions(0, 0));

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstRef,
      Pose2d prevEstimatedRobotPose) {
    poseEstRef.setReferencePose(prevEstimatedRobotPose);
    return poseEstRef.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
