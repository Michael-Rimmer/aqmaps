package uk.ac.ed.inf.aqmaps;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

import com.mapbox.geojson.Point;

/**
 * Utilities class contains helper functions for the main App.
 */

public final class Utilities {

    // Delete file if it exists then write file with provided content parameter
    public static void writeFile(String filePath, String content) {
        final Path file = Path.of(filePath);

        try {
            Files.deleteIfExists(file);
            Files.writeString(file, (CharSequence) content, StandardOpenOption.CREATE);
        } catch (IOException e) {
            System.out.println("Error occured during writing of file: " + filePath + "\n" + e);
            System.exit(1);
        }
    }
    
    public static double euclideanDistance(Point a, Point b) {
        // Computes Euclidean distance between two geojson point objects
        return Math.sqrt(Math.pow(a.longitude()-b.longitude(),2) + Math.pow(a.latitude()-b.latitude(),2));
    }
}
