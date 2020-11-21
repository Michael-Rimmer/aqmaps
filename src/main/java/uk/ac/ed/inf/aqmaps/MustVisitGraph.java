package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

import org.jgrapht.alg.tour.NearestNeighborHeuristicTSP;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.Point;

public class MustVisitGraph {
    // Graph to store locations on the map that must be visited (drone starting location and all sensors)
    // Weighted edges represent the number of move stations between each pair of locations
    private SimpleWeightedGraph<MustVisitLocation, DefaultWeightedEdge> mustVisitGraph = 
            new SimpleWeightedGraph<MustVisitLocation,DefaultWeightedEdge>(DefaultWeightedEdge.class);

    private final MoveStationGraph moveStationGraph;
    
    private final MustVisitLocation droneStart;
    
    public MustVisitGraph(ArrayList<Sensor> sensors, MustVisitLocation droneStart, MoveStationGraph moveStationGraph) {
        this.droneStart = droneStart;
        this.moveStationGraph = moveStationGraph;
        createMustVisitGraph(sensors);
    }

    // Finds cycle of the must visit locations starting and ending at droneStart
    // then returns the order that the locations should be visited in
    public List<MustVisitLocation> computeMustVisitTravelOrder() {
        // Note this is a greedy algorithm therefore may produce different result each time
        var travellingSalesmanPathFinder = new NearestNeighborHeuristicTSP<MustVisitLocation, DefaultWeightedEdge>(droneStart);
        var mustVisitTravelPath = travellingSalesmanPathFinder.getTour(mustVisitGraph);
        return mustVisitTravelPath.getVertexList();
    }
    
    // Returns set of locations that the drone must visit
    public Set<MustVisitLocation> getMustVisitLocations() {
        return mustVisitGraph.vertexSet();
    }
    
    // Calls high level functions necessary to define the graph
    private void createMustVisitGraph(ArrayList<Sensor> sensors) {
        addMustVisitGraphVertices(sensors);
        setMustVisitLocationsClosestMoveStation();
        addMustVisitGraphEdges();
    }
    
    private int computeMustVisitGraphEdgeWeight(MustVisitLocation a, MustVisitLocation b) {
        // Weights determined by length of the shortest path between closest move stations of the two sensors
        return moveStationGraph.computeShortestPathBetweenMoveStations(a.getClosestMoveStation(), b.getClosestMoveStation()).getLength();
    }
    
    // Add edges between every pair of locations in the graph
    private void addMustVisitGraphEdges() {

        var mustVisitLocations = getMustVisitLocations();

        for (var mustVisitA: mustVisitLocations) {
            for (var mustVisitB: mustVisitLocations) {
                // Check edge does not already exist
                if (mustVisitA != mustVisitB && !mustVisitGraph.containsEdge(mustVisitA, mustVisitB)) {
                    var tempEdge = mustVisitGraph.addEdge(mustVisitA, mustVisitB);
                    var tempEdgeWeight = computeMustVisitGraphEdgeWeight(mustVisitA, mustVisitB);
                    mustVisitGraph.setEdgeWeight(tempEdge, tempEdgeWeight);
                }
            }
        }
    }
    
    // For each sensor, set the closestMoveStation attribute to the closest 
    // move station according to Euclidean distance
    private void setMustVisitLocationsClosestMoveStation() {

        // Iterate over all must visit locations
        for (var mustVisit: getMustVisitLocations()) {

            Point closestStation = null;
            double closestDistance = -1;

            // Iterate over all move stations to find the closest
            for (var moveStation : moveStationGraph.getVertexSet()) {

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
            
            if (mustVisit.closestStationIsValid(closestStation)) {
                mustVisit.setClosestMoveStation(closestStation);
            // If closest station is invalid then we must add a new move station ad-hoc
            } else {
                Point validClosestStation = createNewValidClosestMoveStation(mustVisit, closestStation, true);
                mustVisit.setClosestMoveStation(validClosestStation);
            }
        }
    }
    
    // Return a Point with coordinates in straight line of length DRONE_MOVE_DISTANCE and
    // angle degrees away from move station TODO refactor to best class
    private Point createNeighbourMoveStation(Point moveStation, int angle) {
        Double angleInRadians = angle * Math.PI / 180;
        Double neighbour_lat = moveStation.latitude() + (App.DRONE_MOVE_DISTANCE * Math.cos(angleInRadians));
        Double neighbour_long = moveStation.longitude() + (App.DRONE_MOVE_DISTANCE * Math.sin(angleInRadians));
        return Point.fromLngLat(neighbour_long, neighbour_lat);
    }
    
    // Attempts to create a valid closest move station for mustVisit. Adds move station to move station graph if valid.
    // Handles exception when closest move station is invalid, for example out of sensor range.
    private Point createNewValidClosestMoveStation(MustVisitLocation mustVisit, Point closestStation, boolean checkNeighbours) {

        // Create temporary move station in each possible drone direction
        // angle from closestStation
        for (int angle : App.DRONE_DIRECTION_ANGLES) {
            var tempStation = createNeighbourMoveStation(closestStation, angle);
            
            // Skip if move station with same coordinates already exists
            if (moveStationGraph.containsVertex(tempStation)) continue;
            
            // Connect neighbour move station to graph to test if valid
            var tempVertex = moveStationGraph.addVertex(tempStation);
            var tempEdge = moveStationGraph.addEdge(closestStation, tempStation);
            boolean stationIsValid = mustVisit.closestStationIsValid(tempStation);

            // Check if station is valid
            if (stationIsValid && tempVertex != null && tempEdge != null) {
                 return tempVertex;
             } else {
                 // Remove tempStation from moveStationGraph since invalid
                 moveStationGraph.removeEdge(tempEdge);
                 moveStationGraph.removeVertex(tempVertex);
             }
        }
        
        // Failed create new move station from closest station find therefore 
        // attempt to create temporary move station in each possible drone direction
        // angle from each neighbour of closestStation
        if (checkNeighbours) return createNeighbourMoveStations(mustVisit, closestStation);

        // Failed to create valid closest move station
        // Return closestStation despite it being invalid
        return closestStation;
    }
    
    private Point createNeighbourMoveStations(MustVisitLocation mustVisit, Point closestStation) {

        var neighbours = moveStationGraph.getMoveStationNeighbours(closestStation);
//        System.out.println("----- CHECKING " + neighbours.size() + " NEIGHBOURS ----------");
        // Run function on each of the closestStation's neighbours
        int q = 1;
        for (Point neighbour : neighbours) {
//            System.out.println("----- NEIGHBOUR " + q + " ----------");
            Point tempStation = createNewValidClosestMoveStation(mustVisit, neighbour, false);
            if (tempStation != neighbour) {
                return tempStation;
            }
            q++;
        }
        
        return closestStation;
    }
    
    // Add the drone start location and all sensors as vertices in the graph
    private void addMustVisitGraphVertices(ArrayList<Sensor> sensors) {

        // Add all sensors to the graph
        for (var sensor: sensors) {
            mustVisitGraph.addVertex(sensor);
        }

        mustVisitGraph.addVertex(droneStart);
    }
    
//      public ArrayList<Feature> mustVisitGraphToGeojson(String filename) { // TODO REFACTOR
//      
//      var featuresList = new ArrayList<Feature>();
//      
//      for (var mustVisit : mustVisitGraph.vertexSet()) {
//          if (Sensor.class.isInstance(mustVisit)) {
//              var sensorFeature = ((Sensor) mustVisit).getGeojsonFeature();
//              featuresList.add(sensorFeature);
//          }
//      }
//      
//      String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
//      Utilities.writeFile(filename+".geojson", geojson);
//      
//      return featuresList;
//    }
}
