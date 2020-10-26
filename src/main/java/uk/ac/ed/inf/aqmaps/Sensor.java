package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.Point;

// Class representing a single Sensor.
public class Sensor {
    
    private final String location;
    private final float battery;
    private final float reading;
    private Point longLat;
    private Point closestMoveStation;
    private boolean visited;
    
    public Sensor (String wordsLocation, float battery, float reading) {
        this.location = wordsLocation;
        this.battery = battery;
        this.reading = reading;
        this.visited = false;
    }
    
    public void setVisited(boolean visited) {
        this.visited = visited;
    }
    
    private String getMarkerColor() {
        String markerColor = "";
        if (reading >= 0 && reading <32) {
            markerColor = "#00ff00";
        } else if (reading >= 32 && reading < 64) {
            markerColor = "#40ff00";
        } else if (reading >= 64 && reading < 96) {
            markerColor = "#80ff00";
        } else if (reading >= 96 && reading < 128) {
            markerColor = "#c0ff00";
        } else if (reading >= 128 && reading < 160) {
            markerColor = "#ffc000";
        } else if (reading >= 160 && reading < 192) {
            markerColor = "#ff8000";
        } else if (reading >= 192 && reading < 224) {
            markerColor = "#ff4000";
        } else if (reading >= 224 && reading < 256) {
            markerColor = "#ff0000";
        }
        
        if (battery < 10) markerColor = "#000000";
        
        if (!visited) markerColor = "#aaaaaa";

        return markerColor;
    }
    
    private String getMarkerSymbol() {
        String markerSymbol = "";
        if (reading >= 0 && reading < 128) {
            markerSymbol = "lighthouse";
        } else if (reading >= 128 && reading < 256) {
            markerSymbol = "danger";
        } 
        
        if (battery < 10) markerSymbol = "cross";
        
        if (!visited) markerSymbol = "";

        return markerSymbol;
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
    
    public void setLongLat(double longitude, double latitude) {
        this.longLat = Point.fromLngLat(longitude, latitude);
    }
    
    public Point getLongLat() {
        return this.longLat;
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
    
    public Feature getGeojsonFeature() {
        Feature sensorFeature = Feature.fromGeometry(longLat);
        sensorFeature.addStringProperty("location", location);
        sensorFeature.addStringProperty("rbg-string", getMarkerColor());
        sensorFeature.addStringProperty("marker-color", getMarkerColor());
        sensorFeature.addStringProperty("marker-symbol", getMarkerSymbol());
        return sensorFeature;
    }
    

}
