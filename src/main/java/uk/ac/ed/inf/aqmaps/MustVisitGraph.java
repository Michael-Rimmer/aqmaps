package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.jgrapht.alg.tour.NearestNeighborHeuristicTSP;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

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
        var hamiltonianPathFinder = new NearestNeighborHeuristicTSP<MustVisitLocation, DefaultWeightedEdge>(droneStart);
        var sensorsTravelPath = hamiltonianPathFinder.getTour(mustVisitGraph);
        return sensorsTravelPath.getVertexList();
    }
    
    // Returns set of locations that the drone must visit
    // mustVisitLocations already stored in graph therefore no need to 
    // store them as instance variable
    public Set<MustVisitLocation> getMustVisitLocations() {
        return mustVisitGraph.vertexSet();
    }
    
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
            mustVisit.setClosestMoveStation(closestStation);
        }
    }
    
    private void addMustVisitGraphVertices(ArrayList<Sensor> sensors) {

        // Add all sensors to the graph
        for (var sensor: sensors) {
            this.mustVisitGraph.addVertex(sensor);
        }

        this.mustVisitGraph.addVertex(droneStart);
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
    
    // TODO not used 
//  public String mustVisitGraphToGeojson() {
//  
//  var featuresList = new ArrayList<Feature>();
//  
//  for (var mustVisit : mustVisitGraph.vertexSet()) {
//      if (Sensor.class.isInstance(mustVisit)) {
//          var sensorFeature = ((Sensor) mustVisit).getGeojsonFeature();
//          featuresList.add(sensorFeature);
//      }
//  }
//  
////  for (var edge : sensorsGraph.edgeSet()) {
////      var edgeCoords = new ArrayList<Point>(2);
////      edgeCoords.add(sensorsGraph.getEdgeSource(edge).getClosestMoveStation());
////      edgeCoords.add(sensorsGraph.getEdgeTarget(edge).getClosestMoveStation());
////      LineString edgeLineString = LineString.fromLngLats(edgeCoords);
////      Feature edgeFeature = Feature.fromGeometry(edgeLineString);
////      featuresList.add(edgeFeature);
////  }
//  
//  String geojson = FeatureCollection.fromFeatures(featuresList).toJson();
//  return geojson;
//}
}
