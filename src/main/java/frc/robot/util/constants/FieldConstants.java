package frc.robot.util.constants;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Region2d;

import java.util.*;

import com.pathplanner.lib.util.FlippingUtil;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 * 
 * @author Team 6328
 */
public class FieldConstants {
  public static final FieldType fieldType = FieldType.WELDED;

  public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = layout.getFieldLength();
  public static final double fieldWidth = layout.getFieldWidth();
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);
  public static final double coralDiameter = Units.inchesToMeters(4.5);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(
            layout.getTagPose(16).get().getX(),
            0,
            Rotation2d.fromDegrees(90));
    public static final Pose2d opposingCenterFace =
        new Pose2d(
            layout.getTagPose(3).get().getX(),
            fieldWidth,
            Rotation2d.fromDegrees(-90));
  }

  public static class Barge {
    public static final double netWidth = Units.inchesToMeters(40.0);
    public static final double netHeight = Units.inchesToMeters(88.0);

    public static final double cageWidth = Units.inchesToMeters(6.0);
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final double stationLength = Units.inchesToMeters(79.750);
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d leftCenterFace =
        new Pose2d(
            rightCenterFace.getX(),
            fieldWidth - rightCenterFace.getY(),
            Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

    static {
      // Initialize faces
      var aprilTagLayout = layout;
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          var rightBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));
          var leftBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));

          fillRight.put(level, rightBranchPose);
          fillLeft.put(level, leftBranchPose);
          fillRight2d.put(level, rightBranchPose.toPose2d());
          fillLeft2d.put(level, leftBranchPose.toPose2d());
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
        branchPositions2d.add(fillRight2d);
        branchPositions2d.add(fillLeft2d);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final double separation = Units.inchesToMeters(72.0);
    public static final Translation2d[] iceCreams = new Translation2d[3];

    static {
      for (int i = 0; i < 3; i++) {
        iceCreams[i] =
            new Translation2d(
                Units.inchesToMeters(48), fieldWidth / 2.0 - separation + separation * i);
      }
    }
  }

  public enum ReefLevel {
    L1(0, Units.inchesToMeters(25.0), 0),
    L2(1, Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(2, Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(3, Units.inchesToMeters(72), -90);

    ReefLevel(int levelNumber, double height, double pitch) {
      this.levelNumber = levelNumber;
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final int levelNumber;
    public final double height;
    public final double pitch;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;

  public record CoralObjective(int branchId, ReefLevel reefLevel) {}

  
  public enum FieldType {
    ANDYMARK,
    WELDED;
  }

  public enum AlgaePositions {
    HIGH(Regions.REEF_ZONE_2, Regions.REEF_ZONE_4, Regions.REEF_ZONE_6),
    LOW(Regions.REEF_ZONE_1, Regions.REEF_ZONE_3, Regions.REEF_ZONE_5);

    private List<Region2d> regions = new ArrayList<>();

    private AlgaePositions(Region2d... regions) {
      this.regions = Arrays.asList(regions);
    }

    public List<Region2d> getRegions() {
      return regions;
    }
  }

  public class Regions {
    private static final Translation2d[] REEF_ZONE_1_VERTICIES = {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
    };

    private static final Translation2d[] REEF_ZONE_2_VERTICIES = {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
    };

    private static final Translation2d[] REEF_ZONE_3_VERTICIES = {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
    };

    private static final Translation2d[] REEF_ZONE_4_VERTICIES = {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
    };

    private static final Translation2d[] REEF_ZONE_5_VERTICIES = {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
    };

    private static final Translation2d[] REEF_ZONE_6_VERTICIES = {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
        new Translation2d(0.0, 0.0),
    };


    public static final Region2d REEF_ZONE_1 = new Region2d(REEF_ZONE_1_VERTICIES);
    public static final Region2d REEF_ZONE_2 = new Region2d(REEF_ZONE_2_VERTICIES);
    public static final Region2d REEF_ZONE_3 = new Region2d(REEF_ZONE_3_VERTICIES);
    public static final Region2d REEF_ZONE_4 = new Region2d(REEF_ZONE_4_VERTICIES);
    public static final Region2d REEF_ZONE_5 = new Region2d(REEF_ZONE_5_VERTICIES);
    public static final Region2d REEF_ZONE_6 = new Region2d(REEF_ZONE_6_VERTICIES);

    public static void flip() {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        for (Region2d region : getRegions()) {
          region.flip();
        }
      }
    }

    public static Region2d getRegionFromFace(int face) {
      switch (face) {
        case 1:
          return REEF_ZONE_1;
        case 2:
          return REEF_ZONE_2;
        case 3:
          return REEF_ZONE_3;
        case 4:
          return REEF_ZONE_4;
        case 5:
          return REEF_ZONE_5;
        case 6:
          return REEF_ZONE_6;
        default:
          return new Region2d();
      }
    }

    public static Set<Region2d> getRegions() {
        return Set.of(REEF_ZONE_1, REEF_ZONE_2, REEF_ZONE_3, REEF_ZONE_4, REEF_ZONE_5, REEF_ZONE_6);
    }
    
    public static Region2d getRobotRegion(Pose2d robot) {
        for (Region2d region : getRegions()) {
            if (region.contains(robot)) {
                return region;
            }
        }
        return new Region2d();
    }

    public static void flipRegions() {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        for (Region2d region : getRegions()) {
          for (Translation2d vertex : region.getPoints()) {
            FlippingUtil.flipFieldPosition(vertex);
          }
        }
      }
    }
 }
}