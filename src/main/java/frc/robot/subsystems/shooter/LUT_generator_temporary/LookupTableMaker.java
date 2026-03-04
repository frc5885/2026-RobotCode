package frc.robot.subsystems.shooter.LUT_generator_temporary;

import static java.lang.Math.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;
import frc.robot.subsystems.shooter.LUT_generator_temporary.ProjectileSolver.AimData;
import frc.robot.subsystems.shooter.LUT_generator_temporary.ProjectileSolver.Vector;
import java.io.File;
import java.io.IOException;

public class LookupTableMaker {
  public static void writeToJSON(int[][][] array) {
    ObjectMapper mapper = new ObjectMapper();

    ObjectWriter writer = mapper.writerWithDefaultPrettyPrinter();

    try {
      writer.writeValue(
          new File("src\\main\\java\\frc\\robot\\subsystems\\shooter\\trajectorySampleCalcs.json"),
          array);
      System.out.println("Pretty JSON written to trajectorySampleCalcs.json");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static final int[][][] lookupTable = new int[11][24][17];

  public static void makeLookupTable() {
    int nullCount = 0;
    int illegalShotCount = 0;

    int i = 0;

    for (double distance = 1.0; distance <= 6.0; i++, distance += 0.5) {

      int thisNullCount = 0;

      int j = 0;

      for (double angle = -180.0; angle < 180.0; j++, angle += 15.0) {

        int k = 0;

        for (double speed = 0.0; speed <= 4.0; k++, speed += 0.25) {

          Vector velocity = new Vector(speed, angle * PI / 180);

          AimData newTableData = ProjectileSolver.getAimData(distance, velocity);

          if (newTableData == null) {
            nullCount++;
            thisNullCount++;

            lookupTable[i][j][k] = Integer.MAX_VALUE;
          } else {
            if (newTableData.launchAngle * 180 / PI < 45
                || newTableData.launchAngle * 180 / PI > 80) {
              illegalShotCount++;
            }

            // rounds all of the values to one decimal place
            long roundedTurretAngle = round(newTableData.turretAngle * 180 / PI * 10);
            long roundedLaunchAngle = round(newTableData.launchAngle * 180 / PI * 10);
            long roundedVelocity = round(newTableData.velocity * 10);

            int sign = (int) signum(roundedTurretAngle);

            if (sign == 0) sign = 1;

            // compresses the numbers into one variable by assigning them to different parts of a
            // signed 32-bit integer
            long longNewTableDataCompressed =
                (abs(roundedTurretAngle * 1000000) + roundedLaunchAngle * 1000 + roundedVelocity)
                    * sign;
            int newTableDataCompressed = toIntExact(longNewTableDataCompressed);

            // LookupTable.put(newTableIndex, newTableDataCompressed);

            lookupTable[i][j][k] = newTableDataCompressed;
          }
        }
      }

      System.out.println("(" + distance + "): " + thisNullCount);
    }

    System.out.println(nullCount);
    System.out.println(illegalShotCount);

    writeToJSON(lookupTable);
  }
}
