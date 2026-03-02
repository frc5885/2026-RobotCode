package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Zones {
  public static interface Zone {
    public Trigger contains(Supplier<Pose2d> pose);
  }

  public static interface PredictiveXZone extends Zone {
    public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
  }

  public static class BaseZone implements Zone {
    protected final double xMin, xMax, yMin, yMax;

    public BaseZone(double xMin, double xMax, double yMin, double yMax) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
    }

    public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    @Override
    public Trigger contains(Supplier<Pose2d> poseSupplier) {
      return new Trigger(() -> this.containsPoint(poseSupplier.get().getTranslation()));
    }

    protected boolean containsPoint(Translation2d point) {
      return point.getX() >= xMin
          && point.getX() <= xMax
          && point.getY() >= yMin
          && point.getY() <= yMax;
    }

    public BaseZone mirroredX() {
      return new BaseZone(
          FieldConstants.fieldLength - xMax, FieldConstants.fieldLength - xMin, yMin, yMax);
    }

    public BaseZone mirroredY() {
      return new BaseZone(
          xMin, xMax, FieldConstants.fieldWidth - yMax, FieldConstants.fieldWidth - yMin);
    }

    /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
    public Translation2d[] getCorners() {
      return new Translation2d[] {
        new Translation2d(xMin, yMin),
        new Translation2d(xMax, yMin),
        new Translation2d(xMax, yMax),
        new Translation2d(xMin, yMax),
        new Translation2d(xMin, yMin)
      };
    }
  }

  public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {
    public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    public PredictiveXBaseZone(BaseZone baseZone) {
      super(baseZone.xMin, baseZone.xMax, baseZone.yMin, baseZone.yMax);
    }

    public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
    }

    protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
      return (point.getY() >= yMin && point.getY() <= yMax)
          && ((point.getX() >= xMin && point.getX() <= xMax)
              || (point.getX() < xMin
                  && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) >= xMin - point.getX())
              || (point.getX() > xMax
                  && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) <= xMax - point.getX()));
    }

    @Override
    public PredictiveXBaseZone mirroredX() {
      return new PredictiveXBaseZone(super.mirroredX());
    }

    @Override
    public PredictiveXBaseZone mirroredY() {
      return new PredictiveXBaseZone(super.mirroredY());
    }
  }

  public static class ZoneCollection implements Zone {
    protected final Zone[] zones;

    public ZoneCollection(Zone... zones) {
      this.zones = zones;
    }

    @Override
    public Trigger contains(Supplier<Pose2d> pose) {
      Trigger combined = new Trigger(() -> false);

      for (Zone zone : zones) {
        combined = combined.or(zone.contains(pose));
      }

      return combined;
    }
  }

  public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {
    public PredictiveXZoneCollection(PredictiveXZone... zones) {
      super(zones);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      Trigger combined = new Trigger(() -> false);

      for (Zone zone : zones) {
        combined = combined.or(((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt));
      }

      return combined;
    }
  }

  private static final PredictiveXBaseZone blueBottomTrench =
      new PredictiveXBaseZone(
          FieldConstants.LinesVertical.hubCenter
              - (FieldConstants.RightTrench.depth / 2.0)
              - (DriveConstants.fullRobotLength / 2.0),
          FieldConstants.LinesVertical.hubCenter
              + (FieldConstants.RightTrench.depth / 2.0)
              + (DriveConstants.fullRobotLength / 2.0),
          0.0,
          FieldConstants.RightTrench.openingWidth);
  private static final PredictiveXBaseZone blueTopTrench = blueBottomTrench.mirroredY();
  private static final PredictiveXBaseZone redBottomTrench = blueBottomTrench.mirroredX();
  private static final PredictiveXBaseZone redTopTrench = blueTopTrench.mirroredX();

  public static final PredictiveXZoneCollection trenchZones =
      new PredictiveXZoneCollection(blueBottomTrench, blueTopTrench, redBottomTrench, redTopTrench);

  private static final PredictiveXBaseZone blueBottomTrenchDuck =
      new PredictiveXBaseZone(
          FieldConstants.LinesVertical.hubCenter
              - (FieldConstants.RightTrench.barWidth / 2.0)
              - (HoodConstants.extraDuckDistance),
          FieldConstants.LinesVertical.hubCenter
              + (FieldConstants.RightTrench.barWidth / 2.0)
              + (HoodConstants.extraDuckDistance),
          0.0,
          FieldConstants.RightTrench.openingWidth);
  private static final PredictiveXBaseZone blueTopTrenchDuck = blueBottomTrenchDuck.mirroredY();
  private static final PredictiveXBaseZone redBottomTrenchDuck = blueBottomTrenchDuck.mirroredX();
  private static final PredictiveXBaseZone redTopTrenchDuck = blueTopTrenchDuck.mirroredX();

  public static final PredictiveXZoneCollection trenchDuckZones =
      new PredictiveXZoneCollection(
          blueBottomTrenchDuck, blueTopTrenchDuck, redBottomTrenchDuck, redTopTrenchDuck);

  private static final PredictiveXBaseZone blueBottomBump =
      new PredictiveXBaseZone(
          FieldConstants.LinesVertical.hubCenter
              - (FieldConstants.RightBump.depth / 2.0)
              - (DriveConstants.fullRobotLength / 2.0),
          FieldConstants.LinesVertical.hubCenter
              + (FieldConstants.RightBump.depth / 2.0)
              + (DriveConstants.fullRobotLength / 2.0),
          FieldConstants.RightTrench.openingWidth + FieldConstants.RightTrench.blockWidth,
          FieldConstants.RightTrench.openingWidth
              + FieldConstants.RightTrench.blockWidth
              + FieldConstants.RightBump.width);
  private static final PredictiveXBaseZone blueTopBump = blueBottomBump.mirroredY();
  private static final PredictiveXBaseZone redBottomBump = blueBottomBump.mirroredX();
  private static final PredictiveXBaseZone redTopBump = blueTopBump.mirroredX();

  public static final PredictiveXZoneCollection bumpZones =
      new PredictiveXZoneCollection(blueBottomBump, blueTopBump, redBottomBump, redTopBump);

  public static void logAllZones() {
    Logger.recordOutput("Zones/Trenches/Blue Bottom", blueBottomTrench.getCorners());
    Logger.recordOutput("Zones/Trenches/Blue Top", blueTopTrench.getCorners());
    Logger.recordOutput("Zones/Trenches/Red Bottom", redBottomTrench.getCorners());
    Logger.recordOutput("Zones/Trenches/Red Top", redTopTrench.getCorners());

    Logger.recordOutput("Zones/Trenches Duck/Blue Bottom", blueBottomTrenchDuck.getCorners());
    Logger.recordOutput("Zones/Trenches Duck/Blue Top", blueTopTrenchDuck.getCorners());
    Logger.recordOutput("Zones/Trenches Duck/Red Bottom", redBottomTrenchDuck.getCorners());
    Logger.recordOutput("Zones/Trenches Duck/Red Top", redTopTrenchDuck.getCorners());

    Logger.recordOutput("Zones/Bumps/Blue Bottom", blueBottomBump.getCorners());
    Logger.recordOutput("Zones/Bumps/Blue Top", blueTopBump.getCorners());
    Logger.recordOutput("Zones/Bumps/Red Bottom", redBottomBump.getCorners());
    Logger.recordOutput("Zones/Bumps/Red Top", redTopBump.getCorners());
  }
}
