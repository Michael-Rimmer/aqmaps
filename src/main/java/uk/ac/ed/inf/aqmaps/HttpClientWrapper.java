package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Path2D;
import java.lang.reflect.Type;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.reflect.TypeToken;
import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.Polygon;

// Class to obtain, parse and format data from HTTP server
public class HttpClientWrapper {

    private final String port;
    private final String baseUrl;
    private final HttpClient httpClient;

    // String format templates
    private static final String noFlyZonesPath = "/buildings/no-fly-zones.geojson";
    private static final String mapsPathTemplate = "/maps/%s/%s/%s/air-quality-data.json";
    private static final String wordsPathTemplate = "/words/%s/%s/%s/details.json";
    
    public HttpClientWrapper(String port) {
        this.port = port;
        this.baseUrl = String.format("http://localhost:%s", port);
        this.httpClient = HttpClient.newHttpClient();
    }
    
    // Get no-fly-zones geojson from HTTP server and format into a list of Path2D objects
    public Path2D[] getNoFlyZones() throws Exception {
        // Build HTTP request
        final String noFlyZonesUrl = baseUrl + noFlyZonesPath;
        var request = HttpRequest.newBuilder()
              .uri(URI.create(noFlyZonesUrl))
              .build();

        String response = null;

        try {
            // Send HTTP request
            response = this.httpClient.send(request,HttpResponse.BodyHandlers.ofString()).body();
            System.out.println("Successfully obtained noFlyZones from server.");
        } catch (Exception e) {
            throw new Exception(
                    String.format("ERROR: Failed to GET %s from Webserver on port %s \n %s", 
                            noFlyZonesUrl, 
                            port, 
                            e));
        }

        return createNoFlyZonePaths(response);
    }
    
    // Parses geojson to generate a list of Path2D objects representing no fly zones
    private static Path2D[] createNoFlyZonePaths(String geojson) {

        double longitude;
        double latitude;
        
        // Deserialise geojson
        FeatureCollection featureCollection = FeatureCollection.fromJson(geojson);
        var features = featureCollection.features();

        // Initialise array to store noFlyZones
        Path2D[] noFlyZones = new Path2D[features.size()];

        // Iterate over each feature in the feature collection
        int i = 0;
        for (Feature feature : features) {
            // Get geometry for the feature
            Polygon geometry = (Polygon) feature.geometry();
            var pointsList = geometry.coordinates().get(0);

            // Create a new path object for each feature
            Path2D polygon = new Path2D.Double();

            // Add coordinate pairs from geojson geometry to the path object
            for (int j = 0; j < pointsList.size(); j++) {
                latitude = pointsList.get(j).latitude();
                longitude = pointsList.get(j).longitude();
                if (j == 0) {
                    polygon.moveTo(longitude, latitude);
                }
                polygon.lineTo(longitude, latitude);
            }
            polygon.closePath();
            
            // Add new path to noFlyZone array
            noFlyZones[i] = polygon;
            i++;
        }
        
        return noFlyZones;
    }

    // For a given date, get air-quality-data json from HTTP server and format into ArrayList of sensors.
    public ArrayList<Sensor> getAirQualityData(String year, String month, String day) throws Exception {
        var sensors = new ArrayList<Sensor>();

        // Build HTTP request
        final String airQualityDataUrl = baseUrl+String.format(mapsPathTemplate, year, month, day);
        var request = HttpRequest.newBuilder()
              .uri(URI.create(airQualityDataUrl))
              .build();

        try {
            // Send HTTP request
            String response = this.httpClient.send(request,HttpResponse.BodyHandlers.ofString()).body();

            // Use Java Reflection API to deserialize JSON into ArrayList of sensors
            Type sensorListType = new TypeToken<ArrayList<Sensor>>() {}.getType();
            sensors = new Gson().fromJson(response, sensorListType);
            System.out.println("Successfully obtained air-quality-data from server.");
        } catch (Exception e) {
            throw new Exception(
                    String.format("ERROR: Failed to GET %s from Webserver on port %s \n %s", 
                            airQualityDataUrl, 
                            port, 
                            e));
        }

        return setSensorCoords(sensors);
    }

    // Takes an ArrayList of sensors as input. For each sensor, get its coordinates from HTTP server
    // then assigns to its coordinates attribute.
    private ArrayList<Sensor> setSensorCoords(ArrayList<Sensor> sensors) throws Exception {

        for (var sensor: sensors) {
            // Send Http request
            JsonObject responseJson = getW3WLocation(sensor);
            
            // Parse coordinates from response
            JsonObject coords = responseJson.getAsJsonObject("coordinates");
            double latitude = coords.get("lat").getAsDouble();
            double longitude = coords.get("lng").getAsDouble();
            
            // Set coordinates for sensor object
            sensor.setLongLat(longitude, latitude);
        }

        return sensors;
    }
    
    // For a sensor, get what3words location data from HTTP server and parse to JsonObject.
    private JsonObject getW3WLocation(Sensor sensor) throws Exception {
        
        JsonObject responseJson;
        
        // Build HTTP request
        String[] words = sensor.getWordsLocation().split("\\.");
        String wordsUrl = baseUrl + String.format(wordsPathTemplate, words[0], words[1], words[2]);
        var request = HttpRequest.newBuilder()
              .uri(URI.create(wordsUrl))
              .build();
        
        try {
            // Send HTTP request
            String response = this.httpClient.send(request,HttpResponse.BodyHandlers.ofString()).body();
            
            // Parse JSON response
            responseJson = new Gson().fromJson(response, JsonObject.class);
        } catch (Exception e) {
            throw new Exception(
                    String.format("ERROR: Failed to GET %s from Webserver on port %s \n %s", 
                            wordsUrl, 
                            port, 
                            e));
        }
            
        return responseJson;
    }
}
