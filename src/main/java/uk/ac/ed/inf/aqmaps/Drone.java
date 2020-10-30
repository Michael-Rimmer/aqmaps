package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

// Class to represent drone that flies around campus
// Responsible for generating flight path and readings output
public class Drone {

    private final ArrayList<DroneMove> droneMoves;
    private final String flightPathFileName;
    private final String readingsFileName;
    
    public Drone(ArrayList<DroneMove> droneMoves, String day, String month, String year) {
        this.droneMoves = droneMoves;
        this.flightPathFileName = String.format("flightpath-%s-%s-%s.txt", day, month, year);
        this.readingsFileName = String.format("readings-%s-%s-%s.geojson", day, month, year);
    }
    
    public void generateFlightPath() {
        String flightPath = "";
        String line = "";
        
        // Iterate each move and append its log string
        int i = 1;
        for (DroneMove move : droneMoves) {
            line = String.format("%s,%s\n", i, move.getMoveLog());
            flightPath += line;
            i++;
        }

        Utilities.writeFile(flightPathFileName, flightPath);
    }
    
    public void generateReadingsGeojson() {
        
        var featuresList = new ArrayList<Feature>();
        var flightPathCoords = new ArrayList<Point>(droneMoves.size());
        
        for (DroneMove move : droneMoves) {
                flightPathCoords.add(move.getStartLongLat());

                // Add sensor to features list
                Sensor tempSensor = move.getSensor();
                if (tempSensor != null) {
                    featuresList.add(tempSensor.getGeojsonFeature());
                }
        }

        var lastMove = droneMoves.get(droneMoves.size()-1);
        flightPathCoords.add(lastMove.getEndLongLat());

        // Add line indicating drone flight path
        LineString flightPathLine = LineString.fromLngLats(flightPathCoords);
        Feature flightPath = Feature.fromGeometry(flightPathLine);
        flightPath.addStringProperty("name", "drone_flight_path");
        featuresList.add(flightPath);
        
        // Add campus boundary line
        featuresList.add(generateBoundaryLineFeature());
        
        // Convert list of features to features collection
        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();

        Utilities.writeFile(readingsFileName, geojson);
    }
    
    // Generate Geojson for the outer line that surrounds the heatmap
    public static Feature generateBoundaryLineFeature() {
        var boundaryCoords = new ArrayList<Point>(5);
        
        Double minLong = App.BOUNDARY_LONG_LATS.get("minLong");
        Double minLat = App.BOUNDARY_LONG_LATS.get("minLat");
        Double maxLong = App.BOUNDARY_LONG_LATS.get("maxLong");
        Double maxLat = App.BOUNDARY_LONG_LATS.get("maxLat");
        
        // Compute coordinates from min/max long/lat values
        Point bottomLeftCoord = Point.fromLngLat(minLong, minLat);
        Point topRightCoord = Point.fromLngLat(maxLong, maxLat);
        Point topLeftCoord = Point.fromLngLat(minLong, maxLat);
        Point bottomRightCoord = Point.fromLngLat(maxLong, minLat);
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
