package uk.ac.ed.inf.aqmaps;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

import com.mapbox.geojson.Point;

// Class containing helper functions used throughout package
public final class Utilities {

    // Delete file if it exists then write file with provided content parameter
    public static void writeFile(String filePath, String content) {
        final Path file = Path.of(filePath);

        try {
            Files.deleteIfExists(file);
            Files.writeString(file, (CharSequence) content, StandardOpenOption.CREATE);
            System.out.println("Created file: " + filePath);
        } catch (IOException e) {
            System.err.println("Error occured during writing of file: " + filePath + "\n" + e);
            System.exit(1);
        }
    }
    
    // Computes Euclidean distance between two geojson point objects
    public static Double euclideanDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.longitude()-b.longitude(),2) + Math.pow(a.latitude()-b.latitude(),2));
    }
    
    // Validate command line arguments
    public static void validateArgs(String[] args) throws IllegalArgumentException {
        // Example command line args:
        // 15 06 2021 55.9444 -3.1878 5678 9898

        // Validate number of args provided
        if (args.length != 7) {
            throw new IllegalArgumentException("ERROR: Expecting 7 command line arguments. You entered: " + args.length);
        }
        
        validateDate(args[0], args[1], args[2]);
        validateLatLong(args[3], args[4]);

        System.out.println("Command line arguments are valid.");
    }
    
    private static void validateDate(String dayString, String monthString, String yearString) throws IllegalArgumentException {
        int day = Integer.parseInt(dayString);
        int month = Integer.parseInt(monthString);
        int year = Integer.parseInt(yearString);
        
        if (day < 1 || day > 31 || 
                month < 1 || month > 12 ||
                year != 2020 && year != 2021) {
            throw new IllegalArgumentException("ERROR: Invalid date. You entered: " + String.format("%s-%s-%s", day,month,year));
        }
    }
    
    private static void validateLatLong(String latString, String longString) throws IllegalArgumentException {
        // Ensure lat and long within boundaries
        double lat = Double.parseDouble(latString);
        if (lat < App.BOUNDARY_LONG_LATS.get("minLat") || lat > App.BOUNDARY_LONG_LATS.get("maxLat")) {
            throw new IllegalArgumentException(
                    String.format("ERROR: Expecting latitude strictly between %s-%s. You entered: %s", 
                            App.BOUNDARY_LONG_LATS.get("minLat"),
                            App.BOUNDARY_LONG_LATS.get("maxLat"),
                            lat
                            ));
        }
        
        double lng = Double.parseDouble(longString);
        if (lng < App.BOUNDARY_LONG_LATS.get("minLong") || lng > App.BOUNDARY_LONG_LATS.get("maxLong")) {
            throw new IllegalArgumentException(
                    String.format("ERROR: Expecting longitude strictly between %s-%s. You entered: %s", 
                            App.BOUNDARY_LONG_LATS.get("minLong"),
                            App.BOUNDARY_LONG_LATS.get("maxLong"),
                            lng
                            ));
        }
    }
}
