package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultUndirectedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.GraphWalk;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.alg.tour.NearestNeighborHeuristicTSP;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

// Class to compute drone movements to travel to all sensors and return to starting location
// while avoiding no fly zones.
public class DronePathFinder {
    
    // TODO: refactor out of the class when possible
    // boundaryLongLats elements in format: minLong, minLat, maxLong, maxLat
    private final static double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};
    private final static double MAX_DRONE_MOVE_DISTANCE = 0.0003;
    
    private final Point droneStartingPoint;
    public final ArrayList<Sensor> sensors;
    public final Path2D[] noFlyZones;
    private final double[] boundaryLongLats;
    private final Point[][] moveStations;
    
    // Graph object to store move stations and the paths between them
    // Each move station represented as geojson Point object to store its coordinates
    private Graph<Point,DefaultEdge> moveStationsGraph;
    private SimpleWeightedGraph<Sensor,DefaultWeightedEdge> sensorsGraph;
    private Object dronePath;
    
    public ArrayList<Feature> moveStationsGraphToGeojson() {
        
        var featuresList = new ArrayList<Feature>();
        
        for (var moveStation : moveStationsGraph.vertexSet()) {
            var moveStationFeature = Feature.fromGeometry(moveStation);
            featuresList.add(moveStationFeature);
        }
        
        for (var edge : moveStationsGraph.edgeSet()) {
            var edgeCoords = new ArrayList<Point>(2);
            edgeCoords.add(moveStationsGraph.getEdgeSource(edge));
            edgeCoords.add(moveStationsGraph.getEdgeTarget(edge));
            LineString edgeLineString = LineString.fromLngLats(edgeCoords);
            Feature edgeFeature = Feature.fromGeometry(edgeLineString);
            featuresList.add(edgeFeature);
        }
        
//        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
        
        return featuresList;
    }
    
    public DronePathFinder(Point droneStartingPoint, ArrayList<Sensor> sensors, Path2D[] noFlyZones, double[] boundaryLongLats) {
        this.droneStartingPoint = droneStartingPoint;
        this.sensors = sensors;
        this.noFlyZones = noFlyZones;
        this.boundaryLongLats = boundaryLongLats;
        this.moveStations = createMoveStationVertices();
        this.moveStationsGraph = createMoveStationGraph(moveStations);
//        System.out.println(getShortestPathBetweenMoveStations(moveStations[0][0], moveStations[1][10]));
//        System.out.println(getShortestPathBetweenMoveStations(moveStations[0][0], moveStations[1][1]));
//        System.out.println(getShortestPathBetweenMoveStations(moveStations[0][0], moveStations[1][10]));
//        System.out.println(getShortestPathBetweenMoveStations(moveStations[0][0], moveStations[1][1]));
//        System.out.println(getShortestPathBetweenMoveStations(moveStations[0][0], moveStations[1][10]));
//        this.sensorsGraph = createSensorsGraph();
//        computePath();
    }

//    private void computePath() {
//        var  hamiltonianPathFinder = new NearestNeighborHeuristicTSP<Object, DefaultWeightedEdge>(droneStartingPoint);
//        dronePath = hamiltonianPathFinder.getTour(sensorsGraph);
//    }

    private SimpleWeightedGraph<Sensor,DefaultWeightedEdge> createSensorsGraph() {

        var sensorsGraph = new SimpleWeightedGraph<Sensor,DefaultWeightedEdge>(DefaultWeightedEdge.class);

        addClosestMoveStationToSensors();

        // Add all sensors to the graph
        for (var sensor: sensors) {
            sensorsGraph.addVertex(sensor);
        }

        // Add edges between every pair of sensors
        // with weights determined by length of the shortest path between the two sensors
        for (var sensorA: sensors) {
            for (var sensorB: sensors) {
                if (sensorA != sensorB) {
                    var tempEdge = sensorsGraph.addEdge(sensorA, sensorB);
                    System.out.println("sensor A coords: " + sensorA.getCoords());
                    System.out.println("sensor B coords: " + sensorB.getCoords());
                    var tempEdgeWeight = getShortestPathBetweenMoveStations(sensorA.getClosestMoveStation(), sensorB.getClosestMoveStation());
                    System.out.println(tempEdgeWeight.getClass());
//                    sensorsGraph.setEdgeWeight(tempEdge, tempEdgeWeight);
                }
            }
        }

        return sensorsGraph;
    }
    
    private void addClosestMoveStationToSensors() {
        // For each sensor, set the closestMoveStation attribute to the move station
        // according to Euclidean distance
        for (var sensor: sensors) {
            Point closestStation = moveStations[0][0];
            double closestDistance = euclideanDistance(sensor.getCoords(), moveStations[0][0]);
            // Iterate over all move stations to find the closest
            for (int i = 0; i < moveStations.length; i++) {
                for (int j = 0; j < moveStations[0].length; j++) {
                    if (moveStations[i][j] != null) {
                        Double tempClosestDistance = euclideanDistance(sensor.getCoords(), moveStations[i][j]);
                        if (tempClosestDistance < closestDistance) {
                            closestDistance = tempClosestDistance;
                            closestStation = moveStations[i][j];
                        }
                    }
                }
            }
            try {
                sensor.setClosestMoveStation(closestStation);
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }
    
    private double euclideanDistance(Point a, Point b) {
        // Computes Euclidean distance between two geojson point objects
        return Math.sqrt(Math.pow(a.longitude()-b.longitude(),2) + Math.pow(a.latitude()-b.latitude(),2));
    }
    
    
    
    private GraphPath<Point, DefaultEdge> getShortestPathBetweenMoveStations(Point a, Point b) {
        DijkstraShortestPath<Point, DefaultEdge> shortestPathFinder = new DijkstraShortestPath<Point, DefaultEdge>(moveStationsGraph);
//        System.out.println("after dijkstra instance");
//        System.out.println(a);
//        System.out.println(b);
        GraphPath<Point, DefaultEdge> shortestPath = shortestPathFinder.getPath(a, b);
//        System.out.println("after get path");
//        System.out.println(shortestPath);
        return shortestPath;
    }


    private Graph<Point,DefaultEdge> createMoveStationGraph(Point[][] moveStations) {
        // Pre-computing the points on the map that the drone may move to by distributing these points
        // across the campus map in an equilateral triangle pattern. These points represent the move stations.
        // As a result, the drone is constrained to move in directions that are multiples of 60 degrees: 0, 60, 120, 240, 300
        Graph<Point,DefaultEdge> moveStationsGraph = new DefaultUndirectedGraph<>(DefaultEdge.class);
        
        // Add each move station to the graph
        for (int i = 0; i < moveStations.length; i++) {
            for (int j = 0; j < moveStations[0].length; j++) {
                if (moveStations[i][j] != null) {
                    moveStationsGraph.addVertex(moveStations[i][j]);
                }
            }
        }

        // Add edges to connect move stations in an equilateral triangle pattern
        addMoveStationEdges(moveStationsGraph, moveStations);
        return moveStationsGraph;
    }

    // Create predefined set of points that the drone is allowed to fly to.
    // The campus map is divided into a pattern of equally distributed triangles.
    // The vertices of these triangles form the predefined set of points.
    private Point[][] createMoveStationVertices() {

        // Maximum and minimum lat and long values for campus boundary
        final Double minLong = BOUNDARY_LONG_LATS[0];
        final Double minLat = BOUNDARY_LONG_LATS[1];
        final Double maxLong = BOUNDARY_LONG_LATS[2];
        final Double maxLat = BOUNDARY_LONG_LATS[3];

        // Longitude and latitude length for campus map boundary
        final Double longDistance = Math.abs(maxLong-minLong);
        final Double latDistance = Math.abs(maxLat-minLat);

        final double triangleHeight = Math.sqrt(Math.pow(MAX_DRONE_MOVE_DISTANCE,2)-Math.pow(MAX_DRONE_MOVE_DISTANCE/2.0,2));

        // Shape for move stations matrix 
        final int moveStationColumns = (int) Math.floor(longDistance / MAX_DRONE_MOVE_DISTANCE) + 1;
        final int moveStationRows = (int) Math.floor(latDistance / triangleHeight) + 1;

        Point[][] moveStations = new Point[moveStationRows][moveStationColumns];

        // Calculate the coordinates of each move station
        for (int i = 0 ; i < moveStationRows ; i++) {
            // Offset longitude every second row
            Double offset = (i % 2) * MAX_DRONE_MOVE_DISTANCE/2.0;
            for (int j = 0 ; j < moveStationColumns; j++) {
                Double vertexLong = minLong + (j * MAX_DRONE_MOVE_DISTANCE) + offset;
                // Ensure longitude is within boundary after offset
                if (vertexLong > maxLong) {
                    vertexLong = maxLong;
                }
                Double vertexLat = maxLat - i * triangleHeight;
                Point vertex = Point.fromLngLat(vertexLong, vertexLat);
                moveStations[i][j] = vertex;
            }
        }

        // Remove move stations that are within no fly zones or outside the boundary
        Point[][] validMoveStations = removeInvalidMoveStations(moveStations);

        return moveStations;
    }
    
    private Point[][] removeInvalidMoveStations(Point[][] moveStations) {

        // Set invalid move stations to null. Note function performs directly on moveStations input parameter
        for (int i = 0; i < moveStations.length; i++) {
            for (int j = 0; j < moveStations[0].length; j++) {
                for (var noFlyZone : noFlyZones) {

                    // Create to Point2D object allow inter-operability with Path2D contains function.
                    Point2D moveStation2D = new Point2D.Double(moveStations[i][j].longitude(), moveStations[i][j].latitude());

                    // Set move station to null if it is within a no fly zone
                    // TODO: contains condition will fail if move station is on the edge of a no fly zone
                    // so need to ensure that this isnt the case??? define own contains function???
                    if (noFlyZone.contains(moveStation2D)) {
                        moveStations[i][j] = null;
                        break;
                    } 
//                    TODO delete this!!!
//                    if (!this.campusboundary.contains(moveStation2D)) {
//                        System.out.println("OUTSIDE:" + moveStation2D.getX() + "," + moveStation2D.getY());
                          // this triggers but because contains condition fails if point is on the line...
//                    }
                }
            }
        }

        return moveStations;
    }

    private void addMoveStationEdges(Graph<Point, DefaultEdge> moveStationsGraph, Point[][] moveStations) {
        // Add edges graph to connect move stations in an equilateral triangle pattern
        for (int i = 0; i < moveStations.length; i++) {
            for (int j = 0; j < moveStations[0].length - 1; j++) {

                if (moveStations[i][j] != null && moveStations[i][j+1] != null) {
                    moveStationsGraph.addEdge(moveStations[i][j], moveStations[i][j+1]);
                }
                
                if (i % 2 == 1 && 
                        moveStations[i][j] != null && 
                        i != moveStations.length -1) {
                    if (moveStations[i-1][j] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i-1][j]);
                    if (moveStations[i-1][j+1] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i-1][j+1]);
                    if (moveStations[i+1][j] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i+1][j]);
                    if (moveStations[i+1][j+1] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i+1][j+1]);
                }
            }
        }
        removeInvalidMoveStationEdges(moveStationsGraph);
    }
    
    private void removeInvalidMoveStationEdges(Graph<Point, DefaultEdge> moveStationsGraph) {
        
        Area[] noFlyZonesArea = new Area[noFlyZones.length];
        
        for (int i = 0 ; i < noFlyZones.length ; i++) {
            noFlyZonesArea[i] = new Area(noFlyZones[i]);
        }
        
        int j = 0;
        var invalidEdges = new ArrayList<DefaultEdge>();
        for (var edge : moveStationsGraph.edgeSet()) {
            for (var noFlyZoneArea : noFlyZonesArea) {
                var edgePath = convertGraphEdgeToPath2D(moveStationsGraph, edge);
                var edgeArea = new Area(edgePath);
                if (j == 0) System.out.println(edgeArea.getBounds());
                edgeArea.intersect(noFlyZoneArea);
                if (j == 0) System.out.println(edgeArea.getBounds());
                if (!edgeArea.isEmpty()) {
                    System.out.println("removing edge: " + edge);
                    invalidEdges.add(edge);
                    break;
                }
                j++;
            }
        }
        
        // remove from graph outside of iteration structure above
        //  to avoid ConcurrentModificationException
        for ( var invalidEdge : invalidEdges) {
            moveStationsGraph.removeEdge(invalidEdge);
        }
        
    }
    
    // Convert a graph edge to Path2D object
    // Used to determine if a graph edge intersects a no fly zone
    private Path2D convertGraphEdgeToPath2D(Graph<Point, DefaultEdge> graph ,DefaultEdge edge) {
        Path2D edgePath = new Path2D.Double();

        var edgeSource = graph.getEdgeSource(edge);
        var edgeTarget = graph.getEdgeTarget(edge);
        
        edgePath.moveTo(edgeSource.longitude(), edgeSource.latitude());
        edgePath.lineTo(edgeTarget.longitude(),  edgeSource.latitude());
        // Must give some width to the path otherwise cannot compute intersection
        edgePath.lineTo(edgeTarget.longitude() + 0.0001, edgeTarget.latitude() + 0.0001);
        edgePath.lineTo(edgeSource.longitude() + 0.0001, edgeSource.latitude() + 0.0001);
        edgePath.lineTo(edgeSource.longitude(), edgeSource.latitude());
        edgePath.closePath();

        return edgePath;
    }
    
}
