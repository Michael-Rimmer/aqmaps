package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Point;

// Class to represent a single drone move from one point to another
// Sensor instance stores a Sensor only if the drone move ends close to the Sensor
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
    
    public Point getEndLongLat() {
        return endLongLat;
    }
    
    public Sensor getSensor() {
        return sensor;
    }
    
    // Returns formatted string used to generate flightpath file
    public String getMoveLog() {
        // TODO check if should be null or blank?
        String sensorLocation = (sensor == null) ? null : sensor.getWordsLocation();
        
        final String moveLog = String.format("%s,%s,%s,%s,%s", 
                startLongLat.longitude(),
                startLongLat.latitude(),
                endLongLat.longitude(),
                endLongLat.latitude(),
                sensorLocation
                );

        return moveLog;
    }
}
