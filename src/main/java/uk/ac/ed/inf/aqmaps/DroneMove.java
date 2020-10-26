package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;

import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultEdge;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

public class DroneMove {
    
    private final Point startLongLat;
    private final Point endLongLat;
    private final Sensor sensor;

    public DroneMove(Point startLongLat, Point endLongLat, Sensor sensor) {
        this.startLongLat = startLongLat;
        this.endLongLat = endLongLat;
        this.sensor = sensor;
    }
    
    public Point getStartLongLat() {
        return startLongLat;
    }
    
    public Sensor getSensor() {
        return sensor;
    }
    
    public String getMoveLog() {
        
        String sensorLocation = (sensor == null) ? null : sensor.getWordsLocation();
        
        String moveLog = String.format("%s,%s,%s,%s,%s", 
                startLongLat.longitude(),
                startLongLat.latitude(),
                endLongLat.longitude(),
                endLongLat.latitude(),
                sensorLocation
                );

        return moveLog;
    }
    
//    public ArrayList<Feature> getGeojsonFeature() {
//        var featureList = new ArrayList<Feature>(2);
//        
//        Feature droneFlight = GeojsonUtilities.createLineStringFeature(startLongLat, endLongLat);
//        featureList.add(droneFlight);
//                
//        Feature sensorFeature = null;
//        if (sensor != null) {
//            sensorFeature = sensor.getGeojsonFeature();
//            featureList.add(sensorFeature);
//        }
//        
//        return featureList;
//    }
}
