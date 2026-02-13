package frc.robot.Limelights;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Limelights.*;
import frc.robot.Limelights.LimelightHelpers.PoseEstimate;
import frc.robot.Limelights.LimelightHelpers.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */

  private String Limelight;
  private double mountAngle; //degrees
  private double elevation; //cm

  public LimelightSubsystem(String LimelightName) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    Limelight = LimelightName;
    if (Limelight == "limelight-fourtwo") {
      mountAngle = 0;
      elevation = 5;
    } else {
      mountAngle = 15;
      elevation = 5;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePoseEstimation();
    //SmartDashboard.putNumber("newID", getTargetAprilTagID());
  }

  public double getHorizontalAngleOfErrorDegrees(){
    return getTx();
  }

  public double getVerticalAngleOfErrorDegrees(){
    return getTy();
  }


  public double getTx() {
    return LimelightHelpers.getTX(Limelight);
  }

  public double getTy() {
    return LimelightHelpers.getTY(Limelight);
  }

  public double getTa() {
    return LimelightHelpers.getTA(Limelight);
  }

  public double getXDistance(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(Limelight).getX();

  }
  public double getZDistance(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(Limelight).getZ();
    
  }
  public void setPipeline(int pipe){
    LimelightHelpers.setPipelineIndex(Limelight, pipe);
  }

  public int getID(){
    return (int) LimelightHelpers.getFiducialID(Limelight);
  }

  public double getDistanceToTarget(){
    double x = LimelightHelpers.getCameraPose3d_TargetSpace(Limelight).getX();
    double y = LimelightHelpers.getCameraPose3d_TargetSpace(Limelight).getZ();
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public boolean hasTarget(){
    return LimelightHelpers.getTV(Limelight);
  }

  public double getTxAngleRadians(){
    return Units.degreesToRadians(getTx());
  }

  public int getTargetAprilTagID(){
    RawFiducial[] temp =  LimelightHelpers.getRawFiducials(Limelight);
    int id = -1;
    for (RawFiducial fiducial : temp) {

      id = fiducial.id;
      break;
      //TODO: ADD LOGIC LATER TO GET BEST APRILTAG TX OFFSET WISE
      // double txnc = fiducial.txnc;             // X offset (no crosshair)
      // double tync = fiducial.tync;             // Y offset (no crosshair)
      // double ta = fiducial.ta;                 // Target area
      // double distToCamera = fiducial.distToCamera;  // Distance to camera
      // double distToRobot = fiducial.distToRobot;    // Distance to robot
      // double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
    }
    return id;
  }

  public double getEstDistanceFromHub() {
    // Distance from limelight to goal
    // (goalFromFloor - limelightFromFloor) / Math.tan(mountAngle + angleUp)
    
    return (60 - elevation) / Math.tan(mountAngle + getTy());
  }

  public double getPassingRotation() {
    //distance of hub to ramp is (for now) 2m (TODO: FIX)
    return getTx() + 2 * Math.asin(2/(2* Math.sqrt(getTx())));
  }

  //TODO: localization stuff
  public void updatePoseEstimation() {}

}