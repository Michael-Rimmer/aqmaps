package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

public class Drone {

    private final ArrayList<DroneMove> droneMoves;
    
    public Drone(ArrayList<DroneMove> droneMoves) {
        this.droneMoves = droneMoves;
    }
    
    public String generateFlightPath() {
        String flightPath = "";
        String line = "";
        int i = 1;
        for (DroneMove move : droneMoves) {
            
            line = String.format("%s,%s\n", i, move.getMoveLog());
            flightPath += line;
            i++;
        }
        return flightPath;
    }
    
    public String generateReadingsGeojson() {
        
        var featuresList = new ArrayList<Feature>();
        var lineLongLats = new ArrayList<Point>(droneMoves.size());

        for (DroneMove move : droneMoves) {
                // Add drone flight as a single LineString
                lineLongLats.add(move.getStartLongLat()); // warning this might cause bug cause only start lat long

                // Add sensor
                Sensor tempSensor = move.getSensor();
                if (tempSensor != null) {
                    featuresList.add(tempSensor.getGeojsonFeature());
                }
                
        }
        
        // Add line indicating drone flight path
        LineString flightPathLine = LineString.fromLngLats(lineLongLats);
        Feature flightPath = Feature.fromGeometry(flightPathLine);
        flightPath.addStringProperty("name", "drone_flight_path");
        featuresList.add(flightPath);
        
        // Add campus boundary line
        featuresList.add(generateBoundaryLineFeature());
        
        // Convert list of features to features collection
        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
        return geojson;

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
}
