package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.Point;

// Class representing a single Sensor.
public class Sensor {
    
    private final String location;
    private final float battery;
    private final float reading;
    private Point coords;
    private Point closestMoveStation;
    
    public Sensor (String wordsLocation, float battery, float reading) {
        this.location = wordsLocation;
        this.battery = battery;
        this.reading = reading;
    }
    
    public String getWordsLocation() {
        return this.location;
    }
    
    public float getReading() {
        return this.reading;
    }
    
    public float getBattery() {
        return this.battery;
    }
    
    public void setCoords(double longitude, double latitude) {
        this.coords = Point.fromLngLat(longitude, latitude);
    }
    
    public Point getCoords() {
        return this.coords;
    }
   
    public void setClosestMoveStation(Point moveStation) {
        
//        if(euclideanDistance(coords, moveStation) > 0.0003) {
//            throw new Exception("distance > 0.0003: between sensor with coords: " + coords + " and move station: " + moveStation);
//        }
        this.closestMoveStation = moveStation;
    }
    
    private double euclideanDistance(Point a, Point b) {
        // Computes Euclidean distance between two geojson point objects
        return Math.sqrt(Math.pow(a.longitude()-b.longitude(),2) + Math.pow(a.latitude()-b.latitude(),2));
    }
    
    public Point getClosestMoveStation() {
        return this.closestMoveStation;
    }
    
    public Feature generateGeojson() {
        Feature sensorFeature = Feature.fromGeometry(coords);
        return sensorFeature;
    }
    

}
