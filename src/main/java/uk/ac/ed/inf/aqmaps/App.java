package uk.ac.ed.inf.aqmaps;

import java.io.IOException;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;

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
    public static void main( String[] args ) throws IOException, InterruptedException, URISyntaxException
    {
        
           // hamiltonian dev
//        Graph<URI, DefaultEdge> g = new DefaultUndirectedGraph<>(DefaultEdge.class);
//
//        URI google = new URI("http://www.google.com");
//        URI wikipedia = new URI("http://www.wikipedia.org");
//        URI jgrapht = new URI("http://www.jgrapht.org");
//
//        // add the vertices
//        g.addVertex(google);
//        g.addVertex(jgrapht);
//        g.addVertex(wikipedia);
//        
//
//        // add edges to create linking structure
//        g.addEdge(wikipedia, jgrapht);
//        g.addEdge(google, jgrapht);
//        g.addEdge(google, wikipedia);
//        g.addEdge(wikipedia, google);
//        
////        System.out.println(g.vertexSet());
//        
//        NearestNeighborHeuristicTSP pathFinder = new NearestNeighborHeuristicTSP(wikipedia);
//        var tour = pathFinder.getTour(g);
////        var vlist = tour.getVertexList();
//        System.out.println(tour.getStartVertex());
//        System.out.println(tour.getEndVertex());
//        System.out.println(tour.getEdgeList());
        
//        createMoveStationEdges(moveStations);
        HttpClientWrapper clientWrapper = new HttpClientWrapper("80");
        var sensors = clientWrapper.getAirQualityData("2020", "01", "01");
        var noFlyZones = clientWrapper.getNoFlyZones();
        
        double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};
        System.out.println(noFlyZones[0].getBounds());
        double droneStartingLat = Double.parseDouble(args[3]);
        double droneStartingLong = Double.parseDouble(args[4]);
        Point droneStartingPoint = Point.fromLngLat(droneStartingLong, droneStartingLat);
//        var dpf = new DronePathFinder(droneStartingPoint, sensors, noFlyZones, BOUNDARY_LONG_LATS);
        System.out.println(visualizeSensors(sensors));
        
    }
    
    public static String visualizeSensors(ArrayList<Sensor> sensors) {
        var featuresList = new ArrayList<Feature>(sensors.size());

        for (var sensor: sensors) {
            featuresList.add(sensor.generateGeojson());
        }

        return FeatureCollection.fromFeatures(featuresList).toJson();
    }

    
//    public static List<List<Point>> removeInvalidMoveStations(Point[][] moveStations) {
//        var validMoveStations = new ArrayList<List<Point>>();
//        Point[][] validMoveStations = new Point[moveStations.length][moveStations[0].length];
//        
//        for (int i = 0; i < moveStations.length; i++) {
//            for (int j = 0; j < moveStations.length; j++) {
//                if (isMoveStationInNoFlyZone(moveStations[i][j])) {
//                    
//                }
//            }
//        }
//        
//        return validMoveStations;
//    }
    
//    public static boolean isMoveStationInNoFlyZone(Point moveStation) {
//        if (true) {
//            return true;
//        } else {
//            return false;
//        }
//    }
}
