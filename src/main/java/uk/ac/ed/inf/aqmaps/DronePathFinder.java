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
    
    private final MoveStationGraph moveStationGraph;

    private final MustVisitGraph mustVisitGraph;

    public DronePathFinder(MustVisitLocation droneStartingLocation,
            ArrayList<Sensor> sensors, 
            Path2D[] noFlyZones) {

        this.droneStart = droneStartingLocation;
        this.noFlyZones = noFlyZones;

        this.moveStationGraph = new MoveStationGraph(noFlyZones, droneStart);
        this.mustVisitGraph = new MustVisitGraph(sensors, droneStart, moveStationGraph);
    }

//    // Protected for testing TODO delete?
//    protected MoveStationGraph getMoveStationGraph() {
//        return moveStationGraph;
//    }

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

        var mustVisitTravelOrder = mustVisitGraph.computeMustVisitTravelOrder();

        // Iterate over every pair of must visit locations
        for (int i = 0; i < mustVisitTravelOrder.size()-1; i++) {

            var sourceMustVisit = mustVisitTravelOrder.get(i);
            var targetMustVisit = mustVisitTravelOrder.get(i+1);

            // Get shortest move station path for each pair of must visit locations
            var shortestPath = moveStationGraph.computeShortestPathBetweenMoveStations(
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
        var neighbourMoveStation = moveStationGraph.getNeighbourMoveStation(commonMoveStation);

        // Move to neighbour move station
        DroneMove droneMoveAwayFromCommonStation = new DroneMove(commonMoveStation, neighbourMoveStation, null);
        droneMoves.add(droneMoveAwayFromCommonStation);

        // Check if move finishes at target sensor
        Sensor visitedSensor = checkIfSensorVisited(commonMoveStation, targetMustVisit);

        // Return to common moveStation
        DroneMove droneMoveReturnToCommonStation = new DroneMove(neighbourMoveStation, commonMoveStation, visitedSensor);
        droneMoves.add(droneMoveReturnToCommonStation);
    }

    // TODO comment
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
        for (var mustVisit : mustVisitGraph.getMustVisitLocations()) {
            if(!mustVisit.getVisited()) {
                System.out.println(Sensor.class.isInstance(mustVisit));
                return false;
            }
        }

        return true;
    }
}
