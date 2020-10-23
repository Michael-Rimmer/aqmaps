package uk.ac.ed.inf.aqmaps;

import java.io.IOException;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

// used for graph demo
import org.jgrapht.*;
import org.jgrapht.graph.*;
import org.jgrapht.alg.tour.NearestNeighborHeuristicTSP;
import org.jgrapht.nio.*;
import org.jgrapht.nio.dot.*;
import org.jgrapht.traverse.*;

import java.awt.Polygon;
import java.awt.geom.Path2D;
import java.io.*;
import java.net.*;
import java.util.*;

import com.mapbox.geojson.Point;
import com.google.gson.Gson;
import java.lang.reflect.Type;
import com.google.gson.reflect.TypeToken;
import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;

/**
 * Hello world!
 *
 */
public class App 
{
    public static void main( String[] args ) throws IOException, InterruptedException, URISyntaxException
    {
        
        HttpClientWrapper clientWrapper = new HttpClientWrapper("80");
        var sensors = clientWrapper.getAirQualityData("2020", "01", "01");
        var noFlyZones = clientWrapper.getNoFlyZones();
        
        double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};

        double droneStartingLat = Double.parseDouble(args[3]);
        double droneStartingLong = Double.parseDouble(args[4]);
        Point droneStartingPoint = Point.fromLngLat(droneStartingLong, droneStartingLat);
        System.out.println("before dpf");
        var dpf = new DronePathFinder(droneStartingPoint, sensors, noFlyZones, BOUNDARY_LONG_LATS);
        
//        var featuresList = new ArrayList<Feature>(sensors.size());
//        
//        for (var sensor: sensors) {
//            featuresList.add(sensor.generateGeojson());
//        }
//        
       
        
//        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();

//        var featlist = dpf.moveStationsGraphToGeojson();
        var featlist = dpf.sensorsGraphToGeojson();
        
//        var featlist = new ArrayList<Feature>();
        featlist.add(generateBoundaryLineFeature());
        String geojson = FeatureCollection.fromFeatures(featlist).toJson();
        writeFile("this-is-a-test.geojson",geojson);
//        System.out.println(geojson);
//        System.out.println(geojson);
    }
    
    // generate Geojson for the outer line that surrounds the heatmap
    public static Feature generateBoundaryLineFeature() {
        var boundaryCoords = new ArrayList<Point>(5);
        final double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};
        Point bottomLeftCoord = Point.fromLngLat(BOUNDARY_LONG_LATS[0], BOUNDARY_LONG_LATS[1]);
        Point topRightCoord = Point.fromLngLat(BOUNDARY_LONG_LATS[2], BOUNDARY_LONG_LATS[3]);
        Point topLeftCoord = Point.fromLngLat(BOUNDARY_LONG_LATS[0], BOUNDARY_LONG_LATS[3]);
        Point bottomRightCoord = Point.fromLngLat(BOUNDARY_LONG_LATS[2], BOUNDARY_LONG_LATS[1]);
        boundaryCoords.add(topLeftCoord);
        boundaryCoords.add(topRightCoord);
        boundaryCoords.add(bottomRightCoord);
        boundaryCoords.add(bottomLeftCoord);
        boundaryCoords.add(topLeftCoord);

        LineString boundaryLineString = LineString.fromLngLats(boundaryCoords);
        Feature heatmapFeature = Feature.fromGeometry(boundaryLineString);
        heatmapFeature.addStringProperty("name", "heatmap_boundary");

        return heatmapFeature;
    }
    
    public static void writeFile(String filePath, String content) {
        final Path file = Path.of(filePath);
        try {
            Files.writeString(file, (CharSequence) content, StandardOpenOption.CREATE);
        } catch (IOException e) {
            System.out.println("Error occured during writing of file: " + filePath + "\n" + e);
            System.exit(1);
        }
    }


    
//    public static List<List<Point>> removeInvalidMoveStations(Point[][] moveStations) {
//        var validMoveStations = new ArrayList<List<Point>>();
//        Point[][] validMoveStations = new Point[moveStations.length][moveStations[0].length];
//        
//        for (int i = 0; i < moveStations.length; i++) {
//            for (int j = 0; j < moveStations.length; j++) {
//                if (isMoveStationInNoFlyZone(moveStations[i][j])) {
//                    
//                }
//            }
//        }
//        
//        return validMoveStations;
//    }
    
//    public static boolean isMoveStationInNoFlyZone(Point moveStation) {
//        if (true) {
//            return true;
//        } else {
//            return false;
//        }
//    }
}
