package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.Graphs;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultUndirectedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.alg.tour.NearestNeighborHeuristicTSP;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

// Class to generate drone movements that will travel to all sensors and return to starting location
// while avoiding no fly zones.
public class DronePathFinder {

    // Maximum number times the path finder will attempt to computeDroneMoves if 
    // the computed moves are invalid. Necessary due to greedy nature of 
    // path finding algorithm.
    private static final int MAX_COMPUTE_DRONE_MOVE_ATTEMPTS = 3;

    private final MustVisitLocation droneStart;
    private final Path2D[] noFlyZones;

    // Graph to store move stations and the paths between them
    private Graph<Point,DefaultEdge> moveStationGraph = 
            new DefaultUndirectedGraph<>(DefaultEdge.class);

    // Graph to store locations on the map that must be visited (drone starting location and all sensors)
    // Weighted edges represent the number of move stations between each pair of locations
    private SimpleWeightedGraph<MustVisitLocation, DefaultWeightedEdge> mustVisitGraph = 
            new SimpleWeightedGraph<MustVisitLocation,DefaultWeightedEdge>(DefaultWeightedEdge.class);

    // Stores the order that must visit locations should be visited in
    private List<MustVisitLocation> mustVisitTravelOrder;

    public DronePathFinder(MustVisitLocation droneStartingLocation,
            ArrayList<Sensor> sensors, 
            Path2D[] noFlyZones) throws Exception {

        this.droneStart = droneStartingLocation;
        this.noFlyZones = noFlyZones;

        createMoveStationGraph();
        createMustVisitGraph(sensors);
        computeMustVisitTravelOrder();
    }

    protected Graph<Point, DefaultEdge> getMoveStationGraph() {
        return moveStationGraph;
    }
    
    // Returns set of locations that the drone must visit
    // mustVisitLocations already stored in graph therefore no need to 
    // store them as instance variable
    private Set<MustVisitLocation> getMustVisitLocations() {
        return mustVisitGraph.vertexSet();
    }

    // Returns ArrayList of must visit locations that are specifically Sensor 
    // instances sensors already stored in graph therefore no need to 
    // store them as instance variable.
    private ArrayList<Sensor> getSensors() {
        
        var mustVisitLocations = getMustVisitLocations();
        
        // Expect number of sensors = number mustVisitLocations - 1 (given droneStart)
        var sensors = new ArrayList<Sensor>(mustVisitLocations.size()-1);
        
        for ( var mustVisit : mustVisitLocations) {
            if (Sensor.class.isInstance(mustVisit)) {
                sensors.add((Sensor) mustVisit);
            }
        }
        
        return sensors;
    }

    // Main public function of the class.
    // Computes drone 
    public ArrayList<DroneMove> getDroneMoves() {

        ArrayList<DroneMove> droneMoves = new ArrayList<DroneMove>();
        boolean droneMovesValid = false;

        // If drone moves are invalid, retry MAX_COMPUTE_DRONE_MOVE_ATTEMPTS times.
        // Often works on first attempt but necessary because path finding algorithm is greedy.
        int i = 0;
        while (i < MAX_COMPUTE_DRONE_MOVE_ATTEMPTS && !droneMovesValid) {
            droneMoves = computeDroneMoves();
            droneMovesValid = checkDroneMovesValid(droneMoves);
            i++;
        }

        // Logging
        if (droneMovesValid) {
            System.out.println("Successfully computed flight path in " + droneMoves.size() + " moves.");
        } else {
            System.out.println("WARNING: Failed to compute valid drone moves.");
        }
        
        return droneMoves;
    }

    // Return an ArrayList of drone moves representing the drones flight path
    private ArrayList<DroneMove> computeDroneMoves() {

        var droneMoves = new ArrayList<DroneMove>();

        // Iterate over every pair of must visit locations
        for (int i = 0; i < mustVisitTravelOrder.size()-1; i++) {

            var sourceMustVisit = mustVisitTravelOrder.get(i);
            var targetMustVisit = mustVisitTravelOrder.get(i+1);

            // Get shortest move station path for each pair of must visit locations
            var shortestPath = computeShortestPathBetweenMoveStations(
                    sourceMustVisit.getClosestMoveStation(),
                    targetMustVisit.getClosestMoveStation());
            
            // Create a drone move for each edge in the path between them
            createDroneMoveInstances(sourceMustVisit, targetMustVisit, shortestPath, droneMoves);
        }

        return droneMoves;
    }

    // Create a drone move for each pair of move stations in the path between
    // source and target must visit locations
    private void createDroneMoveInstances(MustVisitLocation sourceMustVisit, 
            MustVisitLocation targetMustVisit,
            GraphPath<Point, DefaultEdge> shortestPath, 
            ArrayList<DroneMove> droneMoves) {

        var moveStationTravelOrder = shortestPath.getVertexList();
        
        // Corner case if sensors are accessible from same move station
        if (moveStationTravelOrder.size() == 1) {
            createMovesForSensorsCloseToEachOther(moveStationTravelOrder.get(0), targetMustVisit, droneMoves);
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
        
        // Set droneStart visited to true
        if (!Sensor.class.isInstance(sourceMustVisit)) {
            sourceMustVisit.setVisited(true);
        }
    }

    // Necessary because drone must move then take a reading.
    // If source and target sensor have the same closest move station then create
    private void createMovesForSensorsCloseToEachOther(Point commonMoveStation, MustVisitLocation targetMustVisit, ArrayList<DroneMove> droneMoves) {
        // Find a move station that is close-by to the common move station
        var neighbourMoveStation = Graphs.neighborListOf(moveStationGraph, commonMoveStation).get(0);

        // Move to neighbour move station
        DroneMove droneMoveAwayFromCommonStation = new DroneMove(commonMoveStation, neighbourMoveStation, null);
        droneMoves.add(droneMoveAwayFromCommonStation);

        // Check if move finishes at target sensor
        Sensor visitedSensor = checkIfSensorVisited(commonMoveStation, targetMustVisit);

        // Return to common moveStation
        DroneMove droneMoveReturnToCommonStation = new DroneMove(neighbourMoveStation, commonMoveStation, visitedSensor);
        droneMoves.add(droneMoveReturnToCommonStation);
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
    
    private boolean checkDroneMovesValid(ArrayList<DroneMove> droneMoves) {

        // Check no more than 150 drone moves
        if (droneMoves.size()>150) {
            return false;
        }

        // Check all must visit locations have been visited
        for (var mustVisit : getMustVisitLocations()) {
            if(!mustVisit.getVisited()) {
                System.out.println(Sensor.class.isInstance(mustVisit));
                return false;
            }
        }
        
        return true;
    }
    
    // Finds cycle of the must visit locations starting and ending at droneStart
    // then returns the order that the locations should be visited in
    private void computeMustVisitTravelOrder() {
        // Note this is a greedy algorithm therefore may produce different result each time
        var  hamiltonianPathFinder = new NearestNeighborHeuristicTSP<MustVisitLocation, DefaultWeightedEdge>(droneStart);
        var sensorsTravelPath = hamiltonianPathFinder.getTour(mustVisitGraph);
        this.mustVisitTravelOrder = sensorsTravelPath.getVertexList();
    }

    private void addMustVisitGraphVertices(ArrayList<Sensor> sensors) {

        // Add all sensors to the graph
        for (var sensor: sensors) {
            this.mustVisitGraph.addVertex(sensor);
        }

        this.mustVisitGraph.addVertex(droneStart);
    }
    
    // Add edges between every pair of locations in the graph
    private void addMustVisitGraphEdges() {

        var mustVisitLocations = getMustVisitLocations();

        for (var mustVisitA: mustVisitLocations) {
            for (var mustVisitB: mustVisitLocations) {
                // Check edge does not already exist
                if (mustVisitA != mustVisitB && !this.mustVisitGraph.containsEdge(mustVisitA, mustVisitB)) {
                    var tempEdge = this.mustVisitGraph.addEdge(mustVisitA, mustVisitB);
                    var tempEdgeWeight = computeMustVisitGraphEdgeWeight(mustVisitA, mustVisitB);
                    this.mustVisitGraph.setEdgeWeight(tempEdge, tempEdgeWeight);
                }
            }
        }
    }
    
    private int computeMustVisitGraphEdgeWeight(MustVisitLocation a, MustVisitLocation b) {
        // Weights determined by length of the shortest path between closest move stations of the two sensors
        return computeShortestPathBetweenMoveStations(a.getClosestMoveStation(), b.getClosestMoveStation()).getLength();
    }
    
    private void createMustVisitGraph(ArrayList<Sensor> sensors) {
        addMustVisitGraphVertices(sensors);
        setMustVisitLocationsClosestMoveStation();
        addMustVisitGraphEdges();
    }

    // For each sensor, set the closestMoveStation attribute to the closest 
    // move station according to Euclidean distance
    private void setMustVisitLocationsClosestMoveStation() {

        // Iterate over all must visit locations
        for (var mustVisit: getMustVisitLocations()) {
            
            Point closestStation = null;
            double closestDistance = -1;

            // Iterate over all move stations to find the closest
            for (var moveStation : moveStationGraph.vertexSet()) {

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

    private GraphPath<Point, DefaultEdge> computeShortestPathBetweenMoveStations(Point a, Point b) {
        // Use Dijkstras algorithm
        // Note this is a greedy algorithm therefore may produce different result each time
        var shortestPathFinder = new DijkstraShortestPath<Point, DefaultEdge>(moveStationGraph);
        var shortestPath = shortestPathFinder.getPath(a, b);
        return shortestPath;
    }

    // Pre-computing the points (move stations) on the map that the drone may move to.
    // Move stations are distributed across campus map in an equilateral triangle pattern.
    // This means drone is constrained to move in directions that are multiples of 60 degrees: 0, 60, 120, 240, 300
    private void createMoveStationGraph() {

        this.moveStationGraph = new DefaultUndirectedGraph<>(DefaultEdge.class);

        Point[][] moveStationGrid = createMoveStationsGrid();
        addMoveStationGraphVertices(moveStationGrid);
        addMoveStationsGraphEdges(moveStationGrid);
        
        // Remove any move stations that have no edges
        removeIsolatedMoveStations(moveStationGraph);
    }

    // Returns 2D array of Points storing the coordinates of the move stations
    // that are distributed across campus in an equilateral pattern.
    private Point[][] createMoveStationsGrid() {

        // Delta subtracted or added to campus boundary values to ensure
        // all move stations are strictly inside boundary
        final Double latLongDelta = 0.0001;
        
        // Maximum and minimum lat and long values for campus boundary
        final Double minLong = App.BOUNDARY_LONG_LATS.get("minLong") + latLongDelta;
        final Double minLat = App.BOUNDARY_LONG_LATS.get("minLat") + latLongDelta;
        final Double maxLong = App.BOUNDARY_LONG_LATS.get("maxLong") - latLongDelta;
        final Double maxLat = App.BOUNDARY_LONG_LATS.get("maxLat") - latLongDelta;

        // Longitude and latitude length for campus map boundary
        final Double longDistance = Math.abs(maxLong-minLong);
        final Double latDistance = Math.abs(maxLat-minLat);

        // Determine distance between each row of move stations in the grid
        final double distanceBetweenRows = Math.sqrt(Math.pow(App.MAX_DRONE_MOVE_DISTANCE,2)-Math.pow(App.MAX_DRONE_MOVE_DISTANCE/2.0,2));

        // Compute shape for move stations matrix 
        final int moveStationColumns = (int) Math.floor(longDistance / App.MAX_DRONE_MOVE_DISTANCE) + 1;
        final int moveStationRows = (int) Math.floor(latDistance / distanceBetweenRows) + 1;

        var moveStations = new Point[moveStationRows][moveStationColumns];

        // Calculate the coordinates of each move station
        for (int i = 0 ; i < moveStationRows ; i++) {
            // Offset longitude every second row
            Double offset = (i % 2) * App.MAX_DRONE_MOVE_DISTANCE/2.0;
            for (int j = 0 ; j < moveStationColumns; j++) {
                Double vertexLong = minLong + (j * App.MAX_DRONE_MOVE_DISTANCE) + offset;
                // Ensure longitude is within boundary after offset
                if (vertexLong > maxLong) {
                    vertexLong = maxLong;
                }
                Double vertexLat = maxLat - i * distanceBetweenRows;
                Point vertex = Point.fromLngLat(vertexLong, vertexLat);
                moveStations[i][j] = vertex;
            }
        }

       setInvalidMoveStationsToNull(moveStations);
       return moveStations;
    }

    private Point[][] addMoveStationGraphVertices(Point[][] moveStationGrid) {

        // Add each move station to the graph
        for (int i = 0; i < moveStationGrid.length; i++) {
            for (int j = 0; j < moveStationGrid[0].length; j++) {
                if (moveStationGrid[i][j] != null) {
                    moveStationGraph.addVertex(moveStationGrid[i][j]);
                }
            }
        }
        
        // Add drone starting point as vertex
        this.moveStationGraph.addVertex(droneStart.getLongLat());
        
        return moveStationGrid;
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
    
    
    
    // Invalid stations are those within no fly zones.
    // Note function performs directly on moveStations input parameter.
    private void setInvalidMoveStationsToNull(Point[][] moveStations) {

        // Iterate over all move stations
        for (int i = 0; i < moveStations.length; i++) {
            for (int j = 0; j < moveStations[0].length; j++) {
                // Iterate over all no fly zones
                for (var noFlyZone : noFlyZones) {

                    // Create Point2D object for inter-operability with Path2D contains().
                    Point2D moveStation2D = new Point2D.Double(moveStations[i][j].longitude(), moveStations[i][j].latitude());

                    // Set move station to null if it is strictly inside a no fly zone
                    if (noFlyZone.contains(moveStation2D)) {
                        moveStations[i][j] = null;
                        break;
                    }
                }
            }
        }
    }

    // Adds edges in the map to form paths between move stations
    private void addMoveStationsGraphEdges(Point[][] moveStationGrid) {

        // Connect move stations in the graid
        for (int i = 0; i < moveStationGrid.length; i++) {
            for (int j = 0; j < moveStationGrid[0].length - 1; j++) {

                // Ensure not adding edge to an invalid move station
                // Connect every move station to its rightmost neighbour
                if (moveStationGrid[i][j] != null && moveStationGrid[i][j+1] != null) {
                    this.moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i][j+1]);
                }
                
                // For every second row, connect move stations to the closest two above and closest two below.
                if (i % 2 == 1 && moveStationGrid[i][j] != null) {
                    if (moveStationGrid[i-1][j] != null) this.moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i-1][j]);
                    if (moveStationGrid[i-1][j+1] != null) this.moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i-1][j+1]);
                    // Only connect move station to closest two below if not on last row
                    if (i != moveStationGrid.length -1) {
                        if (moveStationGrid[i+1][j] != null) this.moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i+1][j]);
                        if (moveStationGrid[i+1][j+1] != null) this.moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i+1][j+1]);
                    }
                }
            }
        }

        // Corner case since droneStart is not in moveStationGrid
        connectDroneStartToMoveStations(moveStationGrid);
        removeInvalidMoveStationEdges();
    }
    
    // Adds edges between droneStart move station and other adjacent move stations
    private void connectDroneStartToMoveStations(Point[][] moveStations) {
        for (Point moveStation : this.moveStationGraph.vertexSet()) {
            if (Utilities.euclideanDistance(moveStation, droneStart.getLongLat()) < App.MAX_DRONE_MOVE_DISTANCE) {
                this.moveStationGraph.addEdge(droneStart.getLongLat(), moveStation);
            }
        }
    }
    
    // Remove edges that are within no fly zones
    private void removeInvalidMoveStationEdges() {

        // Convert no fly zones into area objects in order to use intersect()
        Area[] noFlyZonesArea = new Area[noFlyZones.length];
        
        for (int i = 0 ; i < noFlyZones.length ; i++) {
            noFlyZonesArea[i] = new Area(noFlyZones[i]);
        }
        
        var invalidEdges = new ArrayList<DefaultEdge>();

        // Iterate over all edges
        for (var edge : this.moveStationGraph.edgeSet()) {
            // Iterate over all no fly zones
            for (var noFlyZoneArea : noFlyZonesArea) {
                // Must convert to Path2D object in order to use intersect()
                var edgePath = convertMoveStationGraphEdgeToPath2D(edge);
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
            this.moveStationGraph.removeEdge(invalidEdge);
        }
        
    }
    
    // Convert a graph edge to Path2D object
    // Used to determine if a graph edge intersects a no fly zone
    private Path2D convertMoveStationGraphEdgeToPath2D(DefaultEdge edge) {
        Path2D edgePath = new Path2D.Double();

        var edgeSource = this.moveStationGraph.getEdgeSource(edge);
        var edgeTarget = this.moveStationGraph.getEdgeTarget(edge);
        
        edgePath.moveTo(edgeSource.longitude(), edgeSource.latitude());
        edgePath.lineTo(edgeTarget.longitude(),  edgeSource.latitude());
        // Must give some width to the path otherwise cannot compute intersection
        edgePath.lineTo(edgeTarget.longitude() + 0.0001, edgeTarget.latitude() + 0.0001);
        edgePath.lineTo(edgeSource.longitude() + 0.0001, edgeSource.latitude() + 0.0001);
        edgePath.lineTo(edgeSource.longitude(), edgeSource.latitude());
        edgePath.closePath();

        return edgePath;
    }
    
//    TODO delete
//    needed for visualisation in report
    
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
//    
//    public String mustVisitGraphToGeojson() {
//        
//        var featuresList = new ArrayList<Feature>();
//        
//        for (var mustVisit : mustVisitGraph.vertexSet()) {
//            if (Sensor.class.isInstance(mustVisit)) {
//                var sensorFeature = ((Sensor) mustVisit).getGeojsonFeature();
//                featuresList.add(sensorFeature);
//            }
//        }
//        
////        for (var edge : sensorsGraph.edgeSet()) {
////            var edgeCoords = new ArrayList<Point>(2);
////            edgeCoords.add(sensorsGraph.getEdgeSource(edge).getClosestMoveStation());
////            edgeCoords.add(sensorsGraph.getEdgeTarget(edge).getClosestMoveStation());
////            LineString edgeLineString = LineString.fromLngLats(edgeCoords);
////            Feature edgeFeature = Feature.fromGeometry(edgeLineString);
////            featuresList.add(edgeFeature);
////        }
//        
//        String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
//        return geojson;
//    }
    
}
