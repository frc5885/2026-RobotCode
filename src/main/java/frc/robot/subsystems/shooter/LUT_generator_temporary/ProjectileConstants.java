package frc.robot.subsystems.shooter.LUT_generator_temporary;

import static java.lang.Math.*;

public final class ProjectileConstants {
  private ProjectileConstants() {
    throw new AssertionError("Cannot instantiate a constants class");
  }

  // coordinates of the HUB - set to (0, 0)
  public static final ProjectileSolver.CoordinatePair HUBpos =
      new ProjectileSolver.CoordinatePair(0.0, 0.0);

  // physical constants of the fuel
  public static final double M = 0.230; // mass
  public static final double D_A = 1.225; // air density
  public static final double A_S = 0.0225; // surface area
  public static final double C_D = 0.47; // drag coefficient
  public static final double G = 9.81; // force of gravity

  // general constant used in trajectory calculations
  public static final double K = D_A * A_S * C_D / 2.0;

  // commonly used terms in calculations
  public static final double sqrt_k_MG = sqrt(K / (M * G));
  public static final double sqrt_Gk_M = sqrt((G * K) / M);
  public static final double sqrt_M_Gk = sqrt(M / (G * K));
  public static final double M_over_k = M / K;
  public static final double M_over_2k = M / (K * 2.0);
}
