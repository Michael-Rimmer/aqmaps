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
    public static void main( String[] args ) throws Exception
    {
        // Declare constants
        double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};
        double MAX_DRONE_MOVE_DISTANCE = 0.0003;
        
        HttpClientWrapper clientWrapper = new HttpClientWrapper("80");
        
        var noFlyZones = clientWrapper.getNoFlyZones();
        double droneStartingLat = Double.parseDouble(args[3]);
        double droneStartingLong = Double.parseDouble(args[4]);
        System.out.println("DRONE START: " + Double.toString(droneStartingLong) + " " + Double.toString(droneStartingLat));
        MustVisitLocation droneStartingPoint = new MustVisitLocation(Point.fromLngLat(droneStartingLong, droneStartingLat));

//        String[] years = {"2020", "2021"};
//        String[] months = {"01", "02", "03", "04", "05", "06","07","08","09","10","11","12"};
//        String[] days = {"01","02","03","04","05","06","07","08","09","10","11","12","13","14","15","16","17","18","19","20","21","22","23","24","25","26","27","28","29","30","31"};
        
        String[] years = {"2020"};
        String[] months = {"01"};
        String[] days = {"01"};
        
        for (String year : years) {
            for (String month : months) {
                for (String day : days) {
//                    try {
                        var sensors = clientWrapper.getAirQualityData(year, month, day);

                        var dpf = new DronePathFinder(droneStartingPoint, sensors, noFlyZones, BOUNDARY_LONG_LATS, MAX_DRONE_MOVE_DISTANCE);
        
                        var droneMoves = dpf.getDroneMoves();
                        Drone drone = new Drone(droneMoves);
                        String geojson = drone.generateReadingsGeojson();
                        String flightPath = drone.generateFlightPath();
                        
                        String flightPathFileName = String.format("flightpath-%s-%s-%s.txt", day, month, year);
                        String readingsFileName = String.format("readings-%s-%s-%s.geojson", day, month, year);

                        Utilities.writeFile(readingsFileName, geojson);
                        Utilities.writeFile(flightPathFileName, flightPath);
                        
                        System.out.println(String.format("%s-%s-%s : ", day,month,year) + droneMoves.size());
                        
                        
//                    } catch (Exception e) {
//                        
//                        System.out.println("FAILED TO GET " + String.format("%s-%s-%s", day,month,year));
//                        break;
//                    } finally {}
                    

                }
            }
        }
    }
    

}
