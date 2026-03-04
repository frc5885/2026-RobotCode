// package com.robot_trajectory_finder;
// // import static java.lang.Math.*;

package frc.robot.subsystems.shooter.LUT_generator_temporary;
// import static java.lang.Math.*;

// import java.io.File;

// import com.fasterxml.jackson.databind.ObjectMapper;
// import com.fasterxml.jackson.databind.SerializationFeature;

class Main {
  public static void main(String[] args) {
    LookupTableMaker.makeLookupTable();

    //     // ProjectileSolver.CoordinatePair robotPos = new ProjectileSolver.CoordinatePair(-4.0,
    // 2.0);
    //     // ProjectileSolver.Vector robotVelocity = new ProjectileSolver.Vector(2.0, 10.0 * PI /
    // 180);

    //     // ProjectileSolver.AimData aimData = ProjectileSolver.getAimData(robotPos,
    // robotVelocity);

    //     // System.out.println(aimData.velocity);
    //     // System.out.println(aimData.launchAngle * 180/PI);
    //     // System.out.println(aimData.turretAngle * 180/PI);
  }

  // public static void main(String[] args) throws Exception {
  //     ObjectMapper mapper = new ObjectMapper();
  //     mapper.enable(SerializationFeature.INDENT_OUTPUT);

  //     int[][][] test = {
  //         { {1,2,3}, {4,5,6} },
  //         { {7,8,9} }
  //     };

  //     mapper.writeValue(new File("test.json"), test);
  // }
}
