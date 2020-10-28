package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.Graphs;
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
    
    private final MustVisitLocation droneStart;
    public ArrayList<Sensor> sensors; // TODO: possibly get rid because already contained in mustvisitlocations
    public final Path2D[] noFlyZones;
    private final double[] boundaryLongLats;
    private final double maxDroneMoveDistance;
    
    // Graph to store move stations and the paths between them
    // Each move station represented as geojson Point object to store its coordinates
    private Graph<Point,DefaultEdge> moveStationGraph;
    
    // Graph to store locations on the map that must be visited.
    // Note these are either the drone starting location or Sensors
    public ArrayList<MustVisitLocation> mustVisitLocations;
    private SimpleWeightedGraph<MustVisitLocation,DefaultWeightedEdge> mustVisitGraph;
    private List<MustVisitLocation> mustVisitTravelOrder; // TODO: map with must visit locations?
    
    public DronePathFinder(MustVisitLocation droneStartingLocation, ArrayList<Sensor> sensors, Path2D[] noFlyZones, double[] boundaryLongLats, double maxDroneMoveDistance) throws Exception {
        this.droneStart = droneStartingLocation;
        this.sensors = sensors;
        this.noFlyZones = noFlyZones;
        this.boundaryLongLats = boundaryLongLats;
        this.maxDroneMoveDistance = maxDroneMoveDistance;

        // remove this. assignments and act directly in the methods
        this.mustVisitLocations = createMustVisitLocations();
        this.moveStationGraph = createMoveStationGraph();
        this.mustVisitGraph = createMustVisitGraph();
        this.mustVisitTravelOrder = computeMustVisitTravelOrder();
    }
    
    private ArrayList<MustVisitLocation> createMustVisitLocations() {
        var mustVisitLocations = new ArrayList<MustVisitLocation>();
        mustVisitLocations.addAll(sensors);
        mustVisitLocations.add(droneStart);
        return mustVisitLocations;
    }
    
    // For each pair of sensors, find the shortest move station path between them.
    // Create a drone move for each edge in the path between them
    public ArrayList<DroneMove> computeDroneMoves() throws Exception {

        var droneMoves = new ArrayList<DroneMove>();

        // Iterate over every pair of must visit locations
        for (int i = 0; i < mustVisitTravelOrder.size()-1; i++) {

//            if (Sensor.class.isInstance(mustVisitTravelOrder.get(i))) {
//                System.out.println(((Sensor) mustVisitTravelOrder.get(i)).getWordsLocation());
//            }
            
            var sourceMustVisit = mustVisitTravelOrder.get(i);
            var targetMustVisit = mustVisitTravelOrder.get(i+1);

            var shortestPath = computeShortestPathBetweenMoveStations(
                    sourceMustVisit.getClosestMoveStation(),
                    targetMustVisit.getClosestMoveStation());
            
            createDroneMoveInstances(sourceMustVisit, targetMustVisit, shortestPath, droneMoves);
        }
        
        checkDroneMovesValid(droneMoves);
//        if(!checkDroneMovesValid(droneMoves)) {
//            return computeDroneMoves();
//        }
        
        return droneMoves;
    }
    
    // TODO refine parameters, source not needed
    // Create a drone move for each pair of move stations in the path between
    // source and target must visit locations
    private void createDroneMoveInstances(MustVisitLocation sourceMustVisit, MustVisitLocation targetMustVisit,
            GraphPath<Point, DefaultEdge> shortestPath, ArrayList<DroneMove> droneMoves) {
        
        var moveStationTravelOrder = shortestPath.getVertexList();
        
        // Corner case if sensors are accessible from same move station
        if (moveStationTravelOrder.size() == 1) {
            var commonMoveStation = moveStationTravelOrder.get(0);
            var neighbourMoveStation = Graphs.neighborListOf(moveStationGraph, commonMoveStation).get(0);
            
            DroneMove droneMoveAwayFromCommonStation = new DroneMove(sourceMustVisit.getClosestMoveStation(), neighbourMoveStation, null);
            droneMoves.add(droneMoveAwayFromCommonStation);
            
            // Determine if drone move finishes at a sensor
            Sensor visitedSensor = checkIfSensorVisited(commonMoveStation, targetMustVisit);
            
            DroneMove droneMoveReturnToCommonStation = new DroneMove(neighbourMoveStation, commonMoveStation, visitedSensor);
            droneMoves.add(droneMoveReturnToCommonStation);
        }

        // Iterate over every pair of move stations in the path 
        // Won't run if corner case is True
        for (int j = 0 ; j < moveStationTravelOrder.size() - 1 ; j++) {

            Point stationA = moveStationTravelOrder.get(j);
            Point stationB = moveStationTravelOrder.get(j+1);

            // Determine if drone move finishes at a sensor
            Sensor visitedSensor = checkIfSensorVisited(stationB, targetMustVisit);
            
            // Create new DroneMove
            DroneMove tempDroneMove = new DroneMove(stationA, stationB, visitedSensor);
            droneMoves.add(tempDroneMove);
        }
    }
    
    // Returns 
    private Sensor checkIfSensorVisited(Point currentMoveStation, MustVisitLocation targetMustVisit) {
        Sensor visitedSensor = null;
        if (currentMoveStation == targetMustVisit.getClosestMoveStation() && Sensor.class.isInstance(targetMustVisit)) {
            targetMustVisit.setVisited(true);
            visitedSensor = (Sensor) targetMustVisit;
        }
        return visitedSensor;
    }
//    TODO DELETE THIS BELONGS TO DRONE
    public String generateFlightPath(ArrayList<DroneMove> droneMoves) {
        String flightPath = "";
        String line = "";
        int i = 1;
        for (DroneMove move : droneMoves) {
            
            line = String.format("%s,%s\n", i, move.getMoveLog());
            flightPath += line;
            i++;
        }
        return flightPath;
    }
    // TODO DELETE
    public String generateReadingsGeojson(ArrayList<DroneMove> droneMoves) {
        
        var featuresList = new ArrayList<Feature>();
        var lineLongLats = new ArrayList<Point>(droneMoves.size());

        for (DroneMove move : droneMoves) {
                // Add drone flight as a single LineString
                lineLongLats.add(move.getStartLongLat()); // warning this might cause bug cause only start lat long
        }
        
        // Add line indicating drone flight path
        LineString flightPathLine = LineString.fromLngLats(lineLongLats);
        Feature flightPath = Feature.fromGeometry(flightPathLine);
        flightPath.addStringProperty("name", "drone_flight_path");
        featuresList.add(flightPath);
        
        for (var mustVisit : mustVisitGraph.vertexSet()) {
            if (Sensor.class.isInstance(mustVisit)) {
                var sensorFeature = ((Sensor) mustVisit).getGeojsonFeature();
                featuresList.add(sensorFeature);
            }
        }
        
        // Convert list of features to features collection
        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
        return geojson;

    }
    
    private boolean checkDroneMovesValid(ArrayList<DroneMove> droneMoves) throws Exception {
        
//        if (droneMoves.size()>150) {
//            System.out.println("dronemoves too long: " + droneMoves.size());
//            Utilities.writeFile(String.format("FAILED.txt"),generateFlightPath(droneMoves));
//            throw new Exception("DRONE MOVES TOO LONG");
//        }
        
        for (var sensor : sensors) {
            if(!sensor.getVisited()) {
                System.out.println("SENSOR NOT VISITED : " + sensor.getWordsLocation() + "," + sensor.getClosestMoveStation());
                Utilities.writeFile(String.format("FAILED-.txt"),generateFlightPath(droneMoves));
                Utilities.writeFile(String.format("FAILED-.geojson"),generateReadingsGeojson(droneMoves));
                throw new Exception("SENSOR NOT VISITED");
            }
        }
        
        return true;
    }
    
    private List<MustVisitLocation> computeMustVisitTravelOrder() {
        // Instantiate path finder that will compute a closed loop of must visit locations with drone start as the first 
        // and last node
        var  hamiltonianPathFinder = new NearestNeighborHeuristicTSP<MustVisitLocation, DefaultWeightedEdge>(droneStart);
        var sensorsTravelPath = hamiltonianPathFinder.getTour(mustVisitGraph);
        var sensorsTravelOrder = sensorsTravelPath.getVertexList();
        return sensorsTravelOrder;
    }

    private void addMustVisitGraphVertices(SimpleWeightedGraph<MustVisitLocation, DefaultWeightedEdge> sensorsGraph) {
        // Add all sensors to the graph
        for (var sensor: sensors) {
            sensorsGraph.addVertex(sensor);
        }
        
        sensorsGraph.addVertex(droneStart);
    }
    
    private void addMustVisitGraphEdges(SimpleWeightedGraph<MustVisitLocation,DefaultWeightedEdge> sensorsGraph) {
        // Add edges between every pair of sensors
       
        for (var mustVisitA: mustVisitLocations) {
            for (var mustVisitB: mustVisitLocations) {
                // Check edge does not already exist
                if (mustVisitA != mustVisitB && !sensorsGraph.containsEdge(mustVisitA, mustVisitB)) {
                    var tempEdge = sensorsGraph.addEdge(mustVisitA, mustVisitB);
                    var tempEdgeWeight = computeMustVisitGraphEdgeWeight(mustVisitA, mustVisitB);
                    sensorsGraph.setEdgeWeight(tempEdge, tempEdgeWeight);
                }
            }
        }
    }
    
    
    private int computeMustVisitGraphEdgeWeight(MustVisitLocation a, MustVisitLocation b) {
        // Weights determined by length of the shortest path between closest move stations of the two sensors
        return computeShortestPathBetweenMoveStations(a.getClosestMoveStation(), b.getClosestMoveStation()).getLength();
    }
    
    private SimpleWeightedGraph<MustVisitLocation, DefaultWeightedEdge> createMustVisitGraph() {

        var mustVisitGraph = new SimpleWeightedGraph<MustVisitLocation,DefaultWeightedEdge>(DefaultWeightedEdge.class);
        
        addClosestMoveStationToMustVisitLocations();
        addMustVisitGraphVertices(mustVisitGraph);
        addMustVisitGraphEdges(mustVisitGraph);
        
        return mustVisitGraph;
    }

    // For each sensor, set the closestMoveStation attribute to the closest 
    // move station according to Euclidean distance
    private void addClosestMoveStationToMustVisitLocations() {

        var moveStations = moveStationGraph.vertexSet();
        
        // Iterate over all sensors
        for (var mustVisit: mustVisitLocations) {

            Point closestStation = null;
            double closestDistance = -1;

            // Iterate over all move stations to find the closest
            for (var moveStation : moveStations) {

                // Initialise closest station
                if (closestStation == null && closestDistance == -1) {
                    closestStation = moveStation;
                    closestDistance = Utilities.euclideanDistance(mustVisit.getLongLat(), moveStation);
                }

                 Double tempClosestDistance = Utilities.euclideanDistance(mustVisit.getLongLat(), moveStation);

                if (tempClosestDistance < closestDistance) {
                    closestDistance = tempClosestDistance;
                    closestStation = moveStation;
                }
            }
            
            mustVisit.setClosestMoveStation(closestStation);
        }
    }
    
    private boolean checkClosestStationValid(MustVisitLocation mustVisit) {
        
        double distance = Utilities.euclideanDistance(mustVisit.getLongLat(), mustVisit.getClosestMoveStation());
        
        if (distance < maxDroneMoveDistance) {
            return true;
        }
        
        return false;
    }
    
    private GraphPath<Point, DefaultEdge> computeShortestPathBetweenMoveStations(Point a, Point b) {
        // Use Dijkstras algorithm
        var shortestPathFinder = new DijkstraShortestPath<Point, DefaultEdge>(moveStationGraph);
        var shortestPath = shortestPathFinder.getPath(a, b);
        return shortestPath;
    }

    // Pre-computing the points on the map that the drone may move to by distributing these points
    // across the campus map in an equilateral triangle pattern. These points represent the move stations.
    // As a result, the drone is constrained to move in directions that are multiples of 60 degrees: 0, 60, 120, 240, 300
    private Graph<Point,DefaultEdge> createMoveStationGraph() {

        Graph<Point,DefaultEdge> moveStationsGraph = new DefaultUndirectedGraph<>(DefaultEdge.class);

        Point[][] moveStations = createMoveStationsGrid();
        addMoveStationsGraphVertices(moveStationsGraph, moveStations);
        addMoveStationsGraphEdges(moveStationsGraph, moveStations);
        
        // Remove any move stations that have no edges
        removeIsolatedMoveStations(moveStationsGraph);

        return moveStationsGraph;
    }
    
    private Point[][] addMoveStationsGraphVertices(Graph<Point, DefaultEdge> moveStationsGraph, Point[][] moveStations) {
        // Add each move station to the graph
        for (int i = 0; i < moveStations.length; i++) {
            for (int j = 0; j < moveStations[0].length; j++) {
                if (moveStations[i][j] != null) {
                    moveStationsGraph.addVertex(moveStations[i][j]);
                }
            }
        }
        
        // Create vertex the drone starting point
        moveStationsGraph.addVertex(droneStart.getLongLat());
        
        return moveStations;
    }

    // Remove any move stations that have no edges from the graph
    private void removeIsolatedMoveStations(Graph<Point, DefaultEdge> moveStationsGraph) {
       
        var isolatedStations = new ArrayList<Point>();
        
        // Iterate over all move stations in the graph
        for (Point moveStation : moveStationsGraph.vertexSet()) {
            if (moveStationsGraph.degreeOf(moveStation) == 0 ) {
                isolatedStations.add(moveStation);
            }
        }
        
        // Remove from graph outside of iteration structure above
        // to avoid ConcurrentModificationException
        for (var isolatedStation : isolatedStations) {
            moveStationsGraph.removeVertex(isolatedStation);
        }
        
    }
    
    // Create predefined set of points that the drone is allowed to fly to.
    // The campus map is divided into a pattern of equally distributed triangles.
    // The vertices of these triangles form the predefined set of points.
    private Point[][] createMoveStationsGrid() {

        // Maximum and minimum lat and long values for campus boundary
        final Double minLong = boundaryLongLats[0];
        final Double minLat = boundaryLongLats[1];
        final Double maxLong = boundaryLongLats[2];
        final Double maxLat = boundaryLongLats[3];

        // Longitude and latitude length for campus map boundary
        final Double longDistance = Math.abs(maxLong-minLong);
        final Double latDistance = Math.abs(maxLat-minLat);

        final double triangleHeight = Math.sqrt(Math.pow(maxDroneMoveDistance,2)-Math.pow(maxDroneMoveDistance/2.0,2));

        // Shape for move stations matrix 
        final int moveStationColumns = (int) Math.floor(longDistance / maxDroneMoveDistance) + 1;
        final int moveStationRows = (int) Math.floor(latDistance / triangleHeight) + 1;

        var moveStations = new Point[moveStationRows][moveStationColumns];

        // Calculate the coordinates of each move station
        for (int i = 0 ; i < moveStationRows ; i++) {
            // Offset longitude every second row
            Double offset = (i % 2) * maxDroneMoveDistance/2.0;
            for (int j = 0 ; j < moveStationColumns; j++) {
                Double vertexLong = minLong + (j * maxDroneMoveDistance) + offset;
                // Ensure longitude is within boundary after offset
                if (vertexLong > maxLong) {
                    vertexLong = maxLong;
                }
                Double vertexLat = maxLat - i * triangleHeight;
                Point vertex = Point.fromLngLat(vertexLong, vertexLat);
                moveStations[i][j] = vertex;
            }
        }

       return(setInvalidMoveStationsToNull(moveStations));
    }
    
    // Set stations that are within no fly zones or outside the boundary to null
    // Note function performs directly on moveStations input parameter
    private Point[][] setInvalidMoveStationsToNull(Point[][] moveStations) {

        // Iterate over all move stations
        for (int i = 0; i < moveStations.length; i++) {
            for (int j = 0; j < moveStations[0].length; j++) {
                for (var noFlyZone : noFlyZones) {

                    // Create to Point2D object for inter-operability with Path2D contains().
                    Point2D moveStation2D = new Point2D.Double(moveStations[i][j].longitude(), moveStations[i][j].latitude());

                    // Set move station to null if it is within a no fly zone
                    // TODO: contains condition will fail if move station is on the edge of a no fly zone
                    if (noFlyZone.contains(moveStation2D)) {
                        moveStations[i][j] = null;
                        break;
                    }
                }
            }
        }

        return moveStations;
    }

    private void addMoveStationsGraphEdges(Graph<Point, DefaultEdge> moveStationsGraph, Point[][] moveStations) {
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
        
        connectDroneStartLocation(moveStationsGraph, moveStations);
        removeInvalidMoveStationEdges(moveStationsGraph);
    }
    
    private void connectDroneStartLocation(Graph<Point, DefaultEdge> moveStationsGraph, Point[][] moveStations) {
        for (Point moveStation : moveStationsGraph.vertexSet()) {
            if (Utilities.euclideanDistance(moveStation, droneStart.getLongLat()) < maxDroneMoveDistance) {
                moveStationsGraph.addEdge(droneStart.getLongLat(), moveStation);
            }
        }
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
        
        for (var moveStation : moveStationGraph.vertexSet()) {
            var moveStationFeature = Feature.fromGeometry(moveStation);
            featuresList.add(moveStationFeature);
        }
        
        for (var edge : moveStationGraph.edgeSet()) {
            var edgeCoords = new ArrayList<Point>(2);
            edgeCoords.add(moveStationGraph.getEdgeSource(edge));
            edgeCoords.add(moveStationGraph.getEdgeTarget(edge));
            LineString edgeLineString = LineString.fromLngLats(edgeCoords);
            Feature edgeFeature = Feature.fromGeometry(edgeLineString);
            featuresList.add(edgeFeature);
        }
        
        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
        Utilities.writeFile("movestations.geojson", geojson);
        
        return featuresList;
    }
    
    public String mustVisitGraphToGeojson() {
        
        var featuresList = new ArrayList<Feature>();
        
        for (var mustVisit : mustVisitGraph.vertexSet()) {
            if (Sensor.class.isInstance(mustVisit)) {
                var sensorFeature = ((Sensor) mustVisit).getGeojsonFeature();
                featuresList.add(sensorFeature);
            }
        }
        
//        for (var edge : sensorsGraph.edgeSet()) {
//            var edgeCoords = new ArrayList<Point>(2);
//            edgeCoords.add(sensorsGraph.getEdgeSource(edge).getClosestMoveStation());
//            edgeCoords.add(sensorsGraph.getEdgeTarget(edge).getClosestMoveStation());
//            LineString edgeLineString = LineString.fromLngLats(edgeCoords);
//            Feature edgeFeature = Feature.fromGeometry(edgeLineString);
//            featuresList.add(edgeFeature);
//        }
        
        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
        return geojson;
    }
    
}
