package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

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
    public Sensor droneStart;
    
    private final Point droneStartingPoint;
    public ArrayList<Sensor> sensors;
    public final Path2D[] noFlyZones;
    private final double[] boundaryLongLats;
    private final Point[][] moveStations;
    
    // Graph object to store move stations and the paths between them
    // Each move station represented as geojson Point object to store its coordinates
    private Graph<Point,DefaultEdge> moveStationsGraph;
    private SimpleWeightedGraph<Sensor,DefaultWeightedEdge> sensorsGraph;
    private ArrayList<DroneMove> droneMoves;
    private List<Sensor> sensorsTravelOrder;
    
    public DronePathFinder(Point droneStartingPoint, ArrayList<Sensor> sensors, Path2D[] noFlyZones, double[] boundaryLongLats) {
        //TODO delete this just a test
        this.droneStart = new Sensor(null,0,0);
        this.droneStart.setLongLat(droneStartingPoint.longitude(), droneStartingPoint.latitude());
        this.droneStartingPoint = droneStartingPoint;
        this.sensors = sensors;
        this.sensors.add(droneStart);
        this.noFlyZones = noFlyZones;
        this.boundaryLongLats = boundaryLongLats;
        this.moveStations = createMoveStationVertices();
        this.moveStationsGraph = createMoveStationGraph(moveStations);
        this.sensorsGraph = createSensorsGraph();
        this.sensorsTravelOrder = computeSensorsTravelOrder();
        this.droneMoves = computeDroneMoves();
    }
    
    private ArrayList<DroneMove> computeDroneMoves() {

        var droneMoves = new ArrayList<DroneMove>();

        // For each pair of sensors, find the shortest move station path between them
        // Create a drone move for each edge in the path between them
        for (int i = 0; i < sensorsTravelOrder.size()-1; i++) {
            Sensor sourceSensor = sensorsTravelOrder.get(i);
            Sensor targetSensor = sensorsTravelOrder.get(i+1);
            
            var shortestPath = getShortestPathBetweenMoveStations(
                    sourceSensor.getClosestMoveStation(),
                    targetSensor.getClosestMoveStation());
            
            // Create a drone move for each pair of move stations in the path between sensors
            for (int j = 0 ; j < shortestPath.getVertexList().size() - 1 ; j++) {

                Point stationA = shortestPath.getVertexList().get(j);
                Point stationB = shortestPath.getVertexList().get(j+1);

                // Compute whether a drone move ends at a sensor
                Sensor tempSensor = null;
                if (stationB == targetSensor.getClosestMoveStation()) {
                    targetSensor.setVisited(true);
                    tempSensor = targetSensor;
                }
                
                // Create DroneMove object and add to list
                DroneMove tempDroneMove = new DroneMove(stationA, stationB, tempSensor);
                droneMoves.add(tempDroneMove);
            }
        }
        return droneMoves;
    }
    
    public ArrayList<DroneMove> getDroneMoves() {
        return this.droneMoves;
    }

    private List<Sensor> computeSensorsTravelOrder() {
        // Finds the shortest loop of the sensors and returns their order
        // TODO set starting point
        
        var  hamiltonianPathFinder = new NearestNeighborHeuristicTSP<Sensor, DefaultWeightedEdge>(droneStart);
        var sensorsTravelPath = hamiltonianPathFinder.getTour(sensorsGraph);
        var sensorsTravelOrder = sensorsTravelPath.getVertexList();
        return sensorsTravelOrder;
    }

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
                // Check edge does not already exists
                if (sensorA != sensorB && !sensorsGraph.containsEdge(sensorA, sensorB)) {
                    var tempEdge = sensorsGraph.addEdge(sensorA, sensorB);
                    var tempEdgeWeight = getShortestPathBetweenMoveStations(sensorA.getClosestMoveStation(), sensorB.getClosestMoveStation()).getLength();
                    sensorsGraph.setEdgeWeight(tempEdge, tempEdgeWeight);
                }
            }
        }

        return sensorsGraph;
    }
    
    private void addClosestMoveStationToSensors() {
        // For each sensor, set the closestMoveStation attribute to the closest 
        // move station according to Euclidean distance
        for (var sensor: sensors) {
            // Initialise closest station
            Point closestStation = moveStations[0][0];
            double closestDistance = euclideanDistance(sensor.getLongLat(), moveStations[0][0]);
            // Iterate over all move stations to find the closest
            for (int i = 0; i < moveStations.length; i++) {
                for (int j = 0; j < moveStations[0].length; j++) {
                    if (moveStations[i][j] != null) {
                        Double tempClosestDistance = euclideanDistance(sensor.getLongLat(), moveStations[i][j]);
                        // Update closest station and distance
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
        var shortestPathFinder = new DijkstraShortestPath<Point, DefaultEdge>(moveStationsGraph);
        var shortestPath = shortestPathFinder.getPath(a, b);
        
        var vertexList = shortestPath.getVertexList();
        for (var vertex : vertexList) {
            if (!vertexList.contains(a)) {
                System.out.println("FUCKED UP a not in shortest path");
            } 
            if (!vertexList.contains(b)) {
                System.out.println("FUCKED UP b not in shortest path");
            } 
        }
        
        
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
       return(removeInvalidMoveStations(moveStations));
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

                // Ensure not adding edge to an invalid move station
                // Connect every move station to its rightmost neighbour
                if (moveStations[i][j] != null && moveStations[i][j+1] != null) {
                    moveStationsGraph.addEdge(moveStations[i][j], moveStations[i][j+1]);
                }
                
                // For every second row, connect move stations to the closest two above and closest two below.
                if (i % 2 == 1 && moveStations[i][j] != null) {
                    if (moveStations[i-1][j] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i-1][j]);
                    if (moveStations[i-1][j+1] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i-1][j+1]);
                    // Only connect move station to closest two below if not on last row
                    if (i != moveStations.length -1) {
                        if (moveStations[i+1][j] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i+1][j]);
                        if (moveStations[i+1][j+1] != null) moveStationsGraph.addEdge(moveStations[i][j], moveStations[i+1][j+1]);
                    }
                }
            }
        }

        removeInvalidMoveStationEdges(moveStationsGraph);
    }
    
    // Remove edges that are within no fly zones
    private void removeInvalidMoveStationEdges(Graph<Point, DefaultEdge> moveStationsGraph) {

        // Convert no fly zones into area objects in order to use intersect()
        Area[] noFlyZonesArea = new Area[noFlyZones.length];
        
        for (int i = 0 ; i < noFlyZones.length ; i++) {
            noFlyZonesArea[i] = new Area(noFlyZones[i]);
        }
        
        var invalidEdges = new ArrayList<DefaultEdge>();

        // Iterate over all edges
        for (var edge : moveStationsGraph.edgeSet()) {
            // Iterate over all no fly zones
            for (var noFlyZoneArea : noFlyZonesArea) {
                // Must convert to Path2D object in order to use intersect()
                var edgePath = convertGraphEdgeToPath2D(moveStationsGraph, edge);
                var edgeArea = new Area(edgePath);

                edgeArea.intersect(noFlyZoneArea);

                // If area is not empty then we know the edge overlaps no fly zone
                if (!edgeArea.isEmpty()) {
                    invalidEdges.add(edge);
                    // Skip to next edge
                    break;
                }
            }
        }
        
        // Remove from graph outside of iteration structure above
        // to avoid ConcurrentModificationException
        for (var invalidEdge : invalidEdges) {
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
    
    public ArrayList<Feature> sensorsGraphToGeojson() {
        
        var featuresList = new ArrayList<Feature>();
        
        for (var sensor : sensorsGraph.vertexSet()) {
            var moveStationFeature = sensor.getGeojsonFeature();
            featuresList.add(moveStationFeature);
        }
        
//        for (var edge : sensorsGraph.edgeSet()) {
//            var edgeCoords = new ArrayList<Point>(2);
//            edgeCoords.add(sensorsGraph.getEdgeSource(edge).getClosestMoveStation());
//            edgeCoords.add(sensorsGraph.getEdgeTarget(edge).getClosestMoveStation());
//            LineString edgeLineString = LineString.fromLngLats(edgeCoords);
//            Feature edgeFeature = Feature.fromGeometry(edgeLineString);
//            featuresList.add(edgeFeature);
//        }
        
//        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
        
        return featuresList;
    }
    
}
