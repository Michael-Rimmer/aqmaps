package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

// Class to represent drone that travels around campus.
// Responsible for generating flight path and readings output.
public class Drone {

    private final ArrayList<DroneMove> droneMoves;
    private final String flightPathFileName;
    private final String readingsFileName;
    
    public Drone(ArrayList<DroneMove> droneMoves, String day, String month, String year) {
        this.droneMoves = droneMoves;
        this.flightPathFileName = String.format("flightpaths/flightpath-%s-%s-%s.txt", day, month, year); //TODO remove dirs
        this.readingsFileName = String.format("readings/readings-%s-%s-%s.geojson", day, month, year);
    }
    
    // Generate .txt file describing each drone move
    public void generateFlightPath() {
        String flightPath = "";
        String line = "";
        
        // Iterate over each move and append its move log string
        int i = 1;
        for (DroneMove move : droneMoves) {
            line = String.format("%s,%s\n", i, move.getMoveLog());
            flightPath += line;
            i++;
        }

        Utilities.writeFile(flightPathFileName, flightPath);
    }
    
    // Generate .geojson file illustrating each drone flight
    // and readings of sensors
    public void generateReadingsGeojson() {
        
        var featuresList = new ArrayList<Feature>();
        var flightPathCoords = new ArrayList<Point>(droneMoves.size());
        
        // Iterate over drone moves
        for (DroneMove move : droneMoves) {
                // Add drone coordinates
                flightPathCoords.add(move.getStartLongLat());

                // Add visited sensor to features list
                Sensor tempSensor = move.getSensor();
                if (tempSensor != null) {
                    featuresList.add(tempSensor.getGeojsonFeature());
                }
        }

        final var lastMove = droneMoves.get(droneMoves.size()-1);
        flightPathCoords.add(lastMove.getEndLongLat());

        // Add line indicating drone flight path
        final LineString flightPathLine = LineString.fromLngLats(flightPathCoords);
        final Feature flightPath = Feature.fromGeometry(flightPathLine);
        flightPath.addStringProperty("name", "drone_flight_path");
        featuresList.add(flightPath);
        
        // Add campus boundary line
        featuresList.add(generateBoundaryLineFeature());
        
        // Convert list of features to features collection
        final String geojson = FeatureCollection.fromFeatures(featuresList).toJson();

        Utilities.writeFile(readingsFileName, geojson);
    }
    
    // Generate Geojson for the outer line that surrounds the heatmap
    private static Feature generateBoundaryLineFeature() {
        var boundaryCoords = new ArrayList<Point>(5);
        
        // Store boundary coordinates in meaningfully named variables
        final Double minLong = App.BOUNDARY_LONG_LATS.get("minLong");
        final Double minLat = App.BOUNDARY_LONG_LATS.get("minLat");
        final Double maxLong = App.BOUNDARY_LONG_LATS.get("maxLong");
        final Double maxLat = App.BOUNDARY_LONG_LATS.get("maxLat");
        
        // Compute boundary coordinates from min/max long/lat values
        final Point bottomLeftCoord = Point.fromLngLat(minLong, minLat);
        final Point topRightCoord = Point.fromLngLat(maxLong, maxLat);
        final Point topLeftCoord = Point.fromLngLat(minLong, maxLat);
        final Point bottomRightCoord = Point.fromLngLat(maxLong, minLat);
        boundaryCoords.add(topLeftCoord);
        boundaryCoords.add(topRightCoord);
        boundaryCoords.add(bottomRightCoord);
        boundaryCoords.add(bottomLeftCoord);
        boundaryCoords.add(topLeftCoord);

        // Create boundary feature
        final LineString boundaryLineString = LineString.fromLngLats(boundaryCoords);
        final Feature heatmapFeature = Feature.fromGeometry(boundaryLineString);
        heatmapFeature.addStringProperty("name", "heatmap_boundary");

        return heatmapFeature;
    }
}
