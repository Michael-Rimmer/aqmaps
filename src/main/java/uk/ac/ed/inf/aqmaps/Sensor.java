package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Feature;

// Class representing a single Sensor.
public class Sensor extends MustVisitLocation {

    private final String location;
    private final Float battery;
    // reading type String because value may be "null"
    private final String reading;

    public Sensor (String wordsLocation, Float battery, String reading) {
        super();
        this.location = wordsLocation;
        this.battery = battery;
        this.reading = reading;
    }

    // Convert reading from string to float
    // Required due to Java Reflection API used to
    // deserialise json
    private Float getReadingAsFloat() {
        try {
            return Float.parseFloat(this.reading);
        } catch (NumberFormatException e) {
            // Case when reading = "null"
            return (float) -1.0;
        }
    }

    private String getMarkerColor() {
        String markerColor = "";

        float reading = getReadingAsFloat();
        
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
        
        if (!getVisited()) markerColor = "#aaaaaa";

        return markerColor;
    }

    private String getMarkerSymbol() {

        float reading = getReadingAsFloat();
        
        String markerSymbol = "";
        if (reading >= 0 && reading < 128) {
            markerSymbol = "lighthouse";
        } else if (reading >= 128 && reading < 256) {
            markerSymbol = "danger";
        } 
        
        if (battery < 10) markerSymbol = "cross";
        
        if (!getVisited()) markerSymbol = "";

        return markerSymbol;
    }
    
    public String getWordsLocation() {
        return this.location;
    }

    public Feature getGeojsonFeature() {
        Feature sensorFeature = Feature.fromGeometry(getLongLat());
        sensorFeature.addStringProperty("location", location);
        sensorFeature.addStringProperty("rbg-string", getMarkerColor());
        sensorFeature.addStringProperty("marker-color", getMarkerColor());
        sensorFeature.addStringProperty("marker-symbol", getMarkerSymbol());
        return sensorFeature;
    }
    

}
