package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;



public class CreatePathDuplicatesForAuto {
    private static double m = 4;//meters, length of half way through the field (field length/2)
    private static String oldPathName = "";//put the path name here to duplicate, it will go in Generated Flipped Paths
    private static String oldPath;
    private static String newPathName;
    private static String newPath;
    private static String goalPathFolder;
    
    public static void main(String[] args) {
        try {
            oldPathName = args[0];// 0 = old name
            oldPath = "src/main/deploy/pathplanner/paths/" + oldPathName + ".path";
            newPathName = args[1];//1 = new name
            newPath = "src/main/deploy/pathplanner/paths/" + newPathName + ".path";
            Files.deleteIfExists(Path.of(newPath));
            goalPathFolder = args[2];
            // File newFile = new File(newPath);
            // newFile.createNewFile();

            //RN DOESNT DO MUCH TO THE PATH, NEEDS MORE TESTING

            BufferedReader inputFile = new BufferedReader(new FileReader(oldPath));
            double numLines = Files.lines(Paths.get(oldPath)).count();
            String line;
            
            for (int i = 0; i < numLines; i++) {
                double oldPos;
                double newPos;
                double d;
                String p1;
                line = inputFile.readLine();
                System.out.println(line);
                if (line.contains("\"y\": ")) {
                    //y logic
                    // System.out.println("\n\n\n" + line.substring(0, line.indexOf(":") + 1));
                    oldPos = Double.parseDouble(line.substring(line.indexOf(":") + 1));
                    d = Math.abs(m - oldPos);

                    if (oldPos < m) {//right, set left
                        newPos = m + d;
                    } else {//left, set right
                        newPos = m - d;
                    }

                    p1 = line.substring(0, line.indexOf(":") + 1);
                    Files.writeString(Path.of(newPath), p1 + " "  + newPos + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                } else if (line.contains("\"rotationDegrees\": ") || line.contains("\"rotation\": ")) {
                    //rot logic
                    p1 = line.substring(0, line.indexOf(":") + 1);
                    Files.writeString(Path.of(newPath), p1 + " "  + -Double.parseDouble(line.substring(line.indexOf(":") + 1)) + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                } else if (line.contains("\"linkedName\": ")) {
                    //remove link
                    p1 = line.substring(0, line.indexOf(":") + 1);
                    Files.writeString(Path.of(newPath), p1 + " " + null + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                } else if (line.contains("\"folder\": ")) {
                    p1 = line.substring(0, line.indexOf(":") + 1);

                    if (goalPathFolder.equals("")) {
                      Files.writeString(Path.of(newPath), p1 + " "  + "\"Generated Flipped Paths\"," + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    } else {
                      Files.writeString(Path.of(newPath), p1 + " " + goalPathFolder + "," + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    }
                    //remove link
                } else {
                    if (i == numLines - 1) {
                        Files.writeString(Path.of(newPath), line, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    } else {
                        Files.writeString(Path.of(newPath), line + "\n", StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    }
                }
            }



   


            inputFile.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

/*
"y": #
"linkedName": null
"rotationDegrees": #
"rotation": #








{
  "waypoints": [
    {
      "anchor": {
        "x": 4.5307275320970035,
        "y": 7.47
      },
      "prevControl": null,
      "nextControl": {
        "x": 3.976866666666667,
        "y": 7.481599999999999
      },
      "isLocked": false,
      "linkedName": "Trench Left Start"
    },
    {
      "anchor": {
        "x": 3.3016555555555556,
        "y": 7.47
      },
      "prevControl": {
        "x": 3.966788888888889,
        "y": 7.471522222222222
      },
      "nextControl": null,
      "isLocked": false,
      "linkedName": "Comp Left Start End"
    }
  ],
  "rotationTargets": [
    {
      "waypointRelativePos": 0.10999999999999985,
      "rotationDegrees": -90.0
    },
    {
      "waypointRelativePos": 0.9061833688699461,
      "rotationDegrees": -90.0
    },
    {
      "waypointRelativePos": 1.3859275053304887,
      "rotationDegrees": -75.04394937694687
    },
    {
      "waypointRelativePos": 3.2099999999999995,
      "rotationDegrees": -98.64532294343778
    },
    {
      "waypointRelativePos": 3.9499999999999953,
      "rotationDegrees": 0.0
    }
  ],
  "goalEndState": {
    "velocity": 0,
    "rotation": -90.0
  },
  ///////////////////////////////"folder": "Generated Flipped Paths",
  "idealStartingState": {
    "velocity": 0,
    "rotation": -90.0
  },
}
 */
