package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Path2D;
import java.util.ArrayList;

import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultEdge;

import com.mapbox.geojson.Point;

// Class to generate drone movements that will travel to all sensors and return to starting location
// while avoiding no fly zones.
public class DronePathFinder {

    // Maximum number times the path finder will attempt to computeDroneMoves if 
    // the computed moves are invalid. Necessary because path finding algorithm
    // is greedy.
    private static final int MAX_COMPUTE_DRONE_MOVE_ATTEMPTS = 3;

    // Represents the fixed points on the map that the drone may
    // fly to and the flight paths between them
    private final MoveStationGraph moveStationGraph;

    // Represents the locations of drone start position
    // and all sensors that the drone must visit and the 
    // number of drone moves between them
    private final MustVisitGraph mustVisitGraph;

    public DronePathFinder(MustVisitLocation droneStart,
            ArrayList<Sensor> sensors, 
            Path2D[] noFlyZones) {

        this.moveStationGraph = new MoveStationGraph(noFlyZones, droneStart);
        this.mustVisitGraph = new MustVisitGraph(sensors, droneStart, moveStationGraph);
    }

    // Main public function of the class.
    // Computes drone movements required to visit all must visit locations 
    // on the map.
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

    // Return an ArrayList of drone moves representing the drone's flight path
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
    private void createDroneMoveInstances(MustVisitLocation source, 
            MustVisitLocation target,
            GraphPath<Point, DefaultEdge> shortestPath, 
            ArrayList<DroneMove> droneMoves) {

        var moveStationTravelOrder = shortestPath.getVertexList();
        
        // Corner case if sensors are accessible from same move station.
        if (moveStationTravelOrder.size() == 1) {
            createMovesForSensorsCloseToEachOther(moveStationTravelOrder.get(0), target, droneMoves);
        }

        // Iterate over every pair of move stations in the path 
        // Won't run if corner case is True
        for (int j = 0 ; j < moveStationTravelOrder.size() - 1 ; j++) {

            Point stationA = moveStationTravelOrder.get(j);
            Point stationB = moveStationTravelOrder.get(j+1);

            // Determine if drone move finishes at a sensor
            var visitedLocation = checkIfLocationVisited(stationB, target);
            
            // Create new DroneMove
            DroneMove tempDroneMove = new DroneMove(stationA, stationB, visitedLocation);
            droneMoves.add(tempDroneMove);
        }
    }

    // Necessary because drone must move then take a reading.
    private void createMovesForSensorsCloseToEachOther(Point commonMoveStation, 
            MustVisitLocation target, 
            ArrayList<DroneMove> droneMoves) {
        // Find a move station that is a neighbour of the common move station
        var neighbourMoveStation = moveStationGraph.getMoveStationNeighbours(commonMoveStation).get(0);

        // Move to neighbour move station
        DroneMove droneMoveAwayFromCommonStation = new DroneMove(commonMoveStation, neighbourMoveStation, null);
        droneMoves.add(droneMoveAwayFromCommonStation);

        // Check if move finishes at target sensor
        var visitedLocation = checkIfLocationVisited(commonMoveStation, target);

        // Return to common moveStation
        DroneMove droneMoveReturnToCommonStation = new DroneMove(neighbourMoveStation, commonMoveStation, visitedLocation);
        droneMoves.add(droneMoveReturnToCommonStation);
    }

    // Check if currentMoveStation is in proximity to targetMustVisit
    // Returns targetMustVisit if true otherwise null
    private MustVisitLocation checkIfLocationVisited(Point currentMoveStation, MustVisitLocation target) {
        if (currentMoveStation == target.getClosestMoveStation()) {
            // Update visited attribute
            target.setVisited(true);
            return target;
        }
        return null;
    }

    private boolean checkDroneMovesValid(ArrayList<DroneMove> droneMoves) {

        // Check no more than 150 drone moves
        if (droneMoves.size() > 150) {
            System.out.println("WARNING: Too many drone moves: " + droneMoves.size());
            return false;
        }

        // Check all must visit locations have been visited
        for (var mustVisit : mustVisitGraph.getMustVisitLocations()) {
            if(!mustVisit.getVisited()) {
                System.out.println("WARNING: Must visit location not visited: " + mustVisit);
                return false;
            }
        }

        return true;
    }
}
