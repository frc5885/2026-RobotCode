// imports
package frc.robot.subsystems.shooter.LUT_generator_temporary;

import static java.lang.Math.*;
// import java.util.Arrays;
// import java.time.Instant;
// import java.time.Duration;

public class ProjectileSolver {

  // class to store the results from the getTargetAngle method
  public static class TargetVelocity {
    double velocity;
    double error;
    double time;

    TargetVelocity(double velocity, double error, double time) {
      this.velocity = velocity;
      this.error = error;
      this.time = time;
    }
  }

  // private static class VelocityData implements Comparable<VelocityData> {
  //     double velocity;
  //     // double angle;
  //     double time;
  //     // double error;

  //     VelocityData (double velocity, TargetVelocity targetAngle) {
  //         this.velocity = velocity;
  //         // this.angle = targetAngle.angle;
  //         this.time = targetAngle.time;
  //         // this.error = targetAngle.error;
  //     }

  //     @Override

  //     public int compareTo (VelocityData other) {
  //         if (this.velocity > other.velocity) {
  //             return -1;
  //         }
  //         else if (this.velocity < other.velocity) {
  //             return 1;
  //         }
  //         else {
  //             return 0;
  //         }
  //     }
  // }

  // class to store the results from the getAimData method
  public static class AimData {
    double velocity;
    double launchAngle;
    double turretAngle;

    AimData(double velocity, double launchAngle, double turretAngle) {
      this.velocity = velocity;
      this.launchAngle = launchAngle;
      this.turretAngle = turretAngle;
    }
  }

  // class for a coordinate pair
  public static class CoordinatePair implements Comparable<CoordinatePair> {
    double x;
    double y;

    CoordinatePair(double x, double y) {
      this.x = x;
      this.y = y;
    }

    // copying method
    public CoordinatePair(CoordinatePair original) {
      this.x = original.x;
      this.y = original.y;
    }

    @Override
    public int compareTo(CoordinatePair other) {
      if (this.x > other.x) {
        return -1;
      } else if (this.x < other.x) {
        return 1;
      } else {
        if (this.y > other.y) {
          return -1;
        } else if (this.y < other.y) {
          return 1;
        } else {
          return 0;
        }
      }
    }
  }

  // rudimentary class to store vector data
  public static class Vector implements Comparable<Vector> {
    double magnitude;
    double angle;
    double xComponent;
    double yComponent;

    Vector(double magnitude, double angle) {
      this.magnitude = magnitude;
      this.angle = angle;

      this.xComponent = magnitude * cos(angle);
      this.yComponent = magnitude * sin(angle);
    }

    @Override
    public int compareTo(Vector other) {
      if (this.angle > other.angle) {
        return -1;
      } else if (this.angle < other.angle) {
        return 1;
      } else {
        if (this.magnitude > other.magnitude) {
          return -1;
        } else if (this.magnitude < other.magnitude) {
          return 1;
        } else {
          return 0;
        }
      }
    }

    // updates component vectors
    void updateComponentVectors() {
      this.xComponent = magnitude * cos(angle);
      this.yComponent = magnitude * sin(angle);
    }
  }

  // static class that takes in target coordinates, muzzle velocity, and robot velocity and uses it
  // to calculate the launch angle
  public static TargetVelocity getTargetVelocityCoarse(
      double targetX,
      double targetY,
      double angleDegrees,
      double horizontalComponent,
      double startSearchRange,
      double endSearchRange) {

    // System.out.println("targetX " + targetX);
    // System.out.println("targetY " + targetY);
    // System.out.println("angleDegrees " + angleDegrees);
    // System.out.println("startSearchRange " + startSearchRange);
    // System.out.println("endSearchRange " + endSearchRange);

    // converts the angle to radians and calculates the sine and cosine
    double angle = angleDegrees * PI / 180;
    double cosA = cos(angle);
    double sinA = sin(angle);

    // constant values
    final double M = ProjectileConstants.M;
    final double G = ProjectileConstants.G;
    final double k = ProjectileConstants.K;

    // terms commonly used when calculating angles
    final double sqrt_k_MG = ProjectileConstants.sqrt_k_MG;
    final double sqrt_M_Gk = ProjectileConstants.sqrt_M_Gk;
    final double sqrt_Gk_M = ProjectileConstants.sqrt_Gk_M;

    final double M_over_k = ProjectileConstants.M_over_k;
    final double M_over_2k = ProjectileConstants.M_over_2k;

    final double targetXterm = (exp((k * targetX) / M) - 1);

    // list containing the best values
    TargetVelocity[] best = new TargetVelocity[5];
    int bestCount = 0;

    double velocityStep = 0.25;

    // for loop that tests all velocities from the start range to the end range
    for (double velocity = startSearchRange; velocity <= endSearchRange; velocity += velocityStep) {

      // calculates component vectors
      double v_h = velocity * cosA + horizontalComponent;
      double v_v = velocity * sinA;

      // calculates commonly used terms
      double atanTerm = atan(v_v * sqrt_k_MG);
      double vv2Term = (k * v_v * v_v) / (M * G);

      // calculates the time at impact
      double t = (M / (k * v_h)) * targetXterm;

      // these two values represent the time at the apex and at the ground
      double b1 = sqrt_M_Gk * atanTerm;

      double b2 = sqrt_M_Gk * (acosh(sqrt(vv2Term + 1)) + atanTerm);

      // the ball must be travelling down when it reaches the target x, this filters out those
      // values
      if (t < b1 || t > b2) continue;

      // System.out.println("Valid time value from v = " + velocity);

      // this math calculates the y value when the ball reaches the target x
      double const1 = M_over_2k * log((vv2Term) + 1);

      double const2 = t * sqrt_Gk_M - atanTerm;

      double y = -M_over_k * log(cosh(const2));

      // calculates how far off the angle is from the ideal angle
      double error = abs(targetY - (y + const1));

      // packages the important data into an object
      TargetVelocity tv = new TargetVelocity(velocity, error, t);

      // sorts the angle into the top 5 best values

      // automatically inserts the value if there are less than 5 values
      if (bestCount < 5) {
        best[bestCount++] = tv;
      }
      // goes through the top 5 values and replaces the worst one
      else {
        int worstIndex = 0;
        for (int i = 1; i < 5; i++) {
          if (best[i].error > best[worstIndex].error) {
            worstIndex = i;
          }
        }
        if (tv.error < best[worstIndex].error) {
          best[worstIndex] = tv;
        }
      }
    }

    // selects the highest contiguous angle among best
    TargetVelocity chosen = best[0];
    for (int i = 1; i < bestCount; i++) {
      if (best[i].error < chosen.error) {
        chosen = best[i];
      }
    }

    if (chosen == null) {
      // System.out.println(bestCount);
      return null;
    }

    // System.out.println("coarse time: " + chosen.time);
    // System.out.println("coarse velocity: " + chosen.velocity);
    // System.out.println("coarse error: " + chosen.error);

    return chosen;
  }

  public static TargetVelocity getTargetVelocityCoarse(
      double targetX, double targetY, double velocity, double horizontalComponent) {
    return getTargetVelocityCoarse(targetX, targetY, velocity, horizontalComponent, 0.0, 18.0);
  }

  // this method uses the Newton-Raphson method to calculate a precise angle
  public static TargetVelocity getTargetVelocityRefined(
      double targetX, double targetY, double angleDegrees, double horizontalComponent) {

    double angle = angleDegrees * PI / 180.0;

    double vMin = 0.5;
    double vMax = 18.0;

    Double fMin = null;
    Double fMax = null;

    // --- 1. Bracket the root ---
    for (double v = vMin; v <= vMax; v += 0.25) {
      Double f = evaluateResidual(v, angle, targetX, targetY, horizontalComponent);
      if (f == null) continue;

      if (fMin == null) {
        fMin = f;
        vMin = v;
      } else if (fMin * f < 0) {
        vMax = v;
        fMax = f;
        break;
      }
    }

    if (fMin == null || fMax == null) return null;

    double v = 0.5 * (vMin + vMax);

    // --- 2. Hybrid Newton-Bisection ---
    for (int i = 0; i < 25; i++) {

      Double f = evaluateResidual(v, angle, targetX, targetY, horizontalComponent);
      if (f == null) {
        v = 0.5 * (vMin + vMax);
        continue;
      }

      if (abs(f) < 1e-4) {
        double t = computeTime(v, angle, targetX, horizontalComponent);
        return new TargetVelocity(v, abs(f), t);
      }

      double dv = 1e-3;
      Double f1 = evaluateResidual(v + dv, angle, targetX, targetY, horizontalComponent);
      Double f2 = evaluateResidual(v - dv, angle, targetX, targetY, horizontalComponent);

      double vNewton = v;

      if (f1 != null && f2 != null) {
        double df = (f1 - f2) / (2 * dv);
        if (abs(df) > 1e-6) {
          vNewton = v - f / df;
        }
      }

      // If Newton step is unsafe → bisect
      if (vNewton <= vMin || vNewton >= vMax || Double.isNaN(vNewton)) {
        vNewton = 0.5 * (vMin + vMax);
      }

      Double fNew = evaluateResidual(vNewton, angle, targetX, targetY, horizontalComponent);
      if (fNew == null || abs(fNew) > abs(f)) {
        v = 0.5 * (vMin + vMax); // bisection fallback
      } else {
        v = vNewton;
      }

      if (fMin * f < 0) {
        vMax = v;
      } else {
        vMin = v;
        fMin = f;
      }
    }

    double t = computeTime(v, angle, targetX, horizontalComponent);
    return new TargetVelocity(
        v, abs(evaluateResidual(v, angle, targetX, targetY, horizontalComponent)), t);
  }

  private static Double evaluateResidual(
      double velocity, double angle, double targetX, double targetY, double horizontalComponent) {

    final double M = ProjectileConstants.M;
    final double G = ProjectileConstants.G;
    final double k = ProjectileConstants.K;

    final double sqrt_k_MG = ProjectileConstants.sqrt_k_MG;
    final double sqrt_M_Gk = ProjectileConstants.sqrt_M_Gk;
    final double sqrt_Gk_M = ProjectileConstants.sqrt_Gk_M;

    final double M_over_k = ProjectileConstants.M_over_k;
    final double M_over_2k = ProjectileConstants.M_over_2k;

    double cosA = cos(angle);
    double sinA = sin(angle);

    double v_h = velocity * cosA + horizontalComponent;
    double v_v = velocity * sinA;

    if (v_h <= 0) return null;

    double atanTerm = atan(v_v * sqrt_k_MG);
    double vv2Term = (k * v_v * v_v) / (M * G);

    double t = (M / (k * v_h)) * (exp((k * targetX) / M) - 1.0);

    double b1 = sqrt_M_Gk * atanTerm;
    double b2 = sqrt_M_Gk * (acosh(sqrt(vv2Term + 1.0)) + atanTerm);

    if (t < b1 || t > b2) return null;

    double const1 = M_over_2k * log(vv2Term + 1.0);
    double const2 = t * sqrt_Gk_M - atanTerm;

    double y = -M_over_k * log(cosh(const2)) + const1;

    return y - targetY;
  }

  private static double computeTime(
      double velocity, double angle, double targetX, double horizontalComponent) {
    final double M = ProjectileConstants.M;
    final double k = ProjectileConstants.K;

    double v_h = velocity * cos(angle) + horizontalComponent;
    return (M / (k * v_h)) * (exp((k * targetX) / M) - 1.0);
  }

  // uses the robot position and robot velocity to get shooter values
  public static AimData getAimData(double HUBdistance, Vector robotVelocity) {

    // constants
    final double targetY = 1.6088;

    // calculates component vectors of robot velocity
    Vector HUBvelocity = new Vector(robotVelocity.magnitude * cos(robotVelocity.angle), 0);
    Vector perpenVelocity = new Vector(robotVelocity.magnitude * sin(robotVelocity.angle), PI / 2);

    // calculates the angle based on the robot velocity for a static shot
    double staticLaunchAngle = 70 + HUBvelocity.magnitude * 5;

    if (staticLaunchAngle > 80) staticLaunchAngle = 80;
    else if (staticLaunchAngle < 60) staticLaunchAngle = 60;

    double launchAngleRadians = staticLaunchAngle * PI / 180;

    // gets velocity data
    TargetVelocity fuelVelocityData =
        getTargetVelocityRefined(HUBdistance, targetY, staticLaunchAngle, HUBvelocity.magnitude);

    if (fuelVelocityData == null) {
      return null;
    }

    // returns null if error is too large
    if (fuelVelocityData.error > 0.1) {
      return null;
    }
 
    // gets component vectors for fuel velocity
    double fuelVelocityForwardComponent = fuelVelocityData.velocity * cos(launchAngleRadians);
    double fuelVelocityVerticalComponent = fuelVelocityData.velocity * sin(launchAngleRadians);
    double fuelVelocityLateralComponent = perpenVelocity.magnitude * -1;

    // calculates the horizontal angle of the turret relative to the field
    double turretAngle = (atan2(fuelVelocityLateralComponent, fuelVelocityForwardComponent));

    // calculates the new ideal launch angle
    double launchAngle =
        atan2(fuelVelocityVerticalComponent,
            hypot(fuelVelocityForwardComponent, fuelVelocityLateralComponent));

    // calculates the velocity of the ball
    double targetVelocity =
        sqrt(
            fuelVelocityForwardComponent * fuelVelocityForwardComponent
                + fuelVelocityLateralComponent * fuelVelocityLateralComponent
                + fuelVelocityVerticalComponent * fuelVelocityVerticalComponent);

    AimData aimData = new AimData(targetVelocity, launchAngle, turretAngle);

    // System.out.println(fuelVelocityForwardComponent);
    // System.out.println(fuelVelocityLateralComponent);
    // System.out.println(fuelVelocityVerticalComponent + "\n");

    return aimData;
  }

  // static class to calculate arccosh
  static double acosh(double x) {
    return log(x + sqrt(x * x - 1.0));
  }
}
