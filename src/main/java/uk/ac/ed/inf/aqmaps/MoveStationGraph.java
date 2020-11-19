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
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultUndirectedGraph;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

// Class representing all the fixed locations that the drone may 
// travel to and the drone moves between them.
public class MoveStationGraph {

    private int offsetIndex;
    private final Path2D[] noFlyZones;
    
    private final Area[] noFlyZoneAreas;
    
    private final MustVisitLocation droneStart;

    // Graph to store move stations and the paths between them
    private Graph<Point,DefaultEdge> moveStationGraph = 
            new DefaultUndirectedGraph<>(DefaultEdge.class);

    public MoveStationGraph(Path2D[] noFlyZones, MustVisitLocation droneStart) {
        this.noFlyZones = noFlyZones;
        this.noFlyZoneAreas = generateNoFlyZoneAreas();
        this.droneStart = droneStart;
        createMoveStationGraph();
    }
    
    private Area[] generateNoFlyZoneAreas() {
        Area[] noFlyZoneAreas = new Area[noFlyZones.length];
        
        for (int i = 0 ; i < noFlyZones.length ; i++) {
            noFlyZoneAreas[i] = new Area(noFlyZones[i]);
        }
        
        return noFlyZoneAreas;
    }
    
    public GraphPath<Point, DefaultEdge> computeShortestPathBetweenMoveStations(Point a, Point b) {
        // Use Dijkstras algorithm
        // Note this is a greedy algorithm therefore may produce different result each time
        var shortestPathFinder = new DijkstraShortestPath<Point, DefaultEdge>(moveStationGraph);
        var shortestPath = shortestPathFinder.getPath(a, b);
        return shortestPath;
    }
    
    // TODO delete
    public boolean containsVertex(Point v) {
        return moveStationGraph.containsVertex(v);
    }
    
    // Returns a move station that is connected by an edge to the given move station
    public List<Point> getMoveStationNeighbours(Point station) {
        return Graphs.neighborListOf(moveStationGraph, station);
    }
    
    // Return vertices in the graph (all move stations)
    public Set<Point> getVertexSet() {
        return moveStationGraph.vertexSet();
    }

    // Calls high level functions necessary to define the graph
    private void createMoveStationGraph() {
        Point[][] moveStationGrid = createMoveStationsGrid();
        addMoveStationGraphVertices(moveStationGrid);
        addMoveStationGraphEdges(moveStationGrid);
        moveStationsGraphToGeojson("allmovestations");
        // Remove any move stations that have no edges
        removeIsolatedMoveStations();
        
    }

    // Remove any move stations that have no edges from the graph
    public void removeIsolatedMoveStations() {
       
        var isolatedStations = new ArrayList<Point>();
        
        // Iterate over all move stations in the graph
        for (Point moveStation : moveStationGraph.vertexSet()) {
            if (moveStationGraph.degreeOf(moveStation) == 0 ) {
                isolatedStations.add(moveStation);
            }
        }
        
        // Remove from graph outside of iteration structure above
        // to avoid ConcurrentModificationException
        for (var isolatedStation : isolatedStations) {
            moveStationGraph.removeVertex(isolatedStation);
        }
        
    }
    
    // Adds edges between droneStart move station and its adjacent move stations
    private void connectDroneStartToMoveStations(Point[][] moveStations) {
        for (Point moveStation : moveStationGraph.vertexSet()) {
            if (Utilities.euclideanDistance(moveStation, droneStart.getLongLat()) < App.DRONE_MOVE_DISTANCE) {
                moveStationGraph.addEdge(droneStart.getLongLat(), moveStation);
            }
        }
    }
    
    // Adds edges in the graph to form paths between move stations
    private void addMoveStationGraphEdges(Point[][] moveStationGrid) {

        // Connect move stations in the grid
        for (int i = 0; i < moveStationGrid.length; i++) {
            for (int j = 0; j < moveStationGrid[0].length - 1; j++) {

                // Connect every move station to its rightmost neighbour
                addEdge(moveStationGrid[i][j], moveStationGrid[i][j+1]);
                
                // For every offsetIndex-th row, connect move stations to the 
                // closest two above and closest two below.
                if ((i+this.offsetIndex) % 2 == 1) {
                    // Only connect move station to closest two above if not on top row
                    if (i != 0) {
                        addEdge(moveStationGrid[i][j], moveStationGrid[i-1][j]);
                        addEdge(moveStationGrid[i][j], moveStationGrid[i-1][j+1]);
                    }
                    // Only connect move station to closest two below if not on last row
                    if (i != moveStationGrid.length -1) {
                        addEdge(moveStationGrid[i][j], moveStationGrid[i+1][j]);
                        addEdge(moveStationGrid[i][j], moveStationGrid[i+1][j+1]);
                    }
                }
            }
        }

        // Corner case since droneStart is not in moveStationGrid TODO
        connectDroneStartToMoveStations(moveStationGrid);
//        removeInvalidMoveStationEdges();
    }
    
    // Remove edges that are within no fly zones TODO delete
    private void removeInvalidMoveStationEdges() {

        // Convert no fly zones into area objects in order to use intersect()
        Area[] noFlyZonesArea = new Area[noFlyZones.length];
        
        for (int i = 0 ; i < noFlyZones.length ; i++) {
            noFlyZonesArea[i] = new Area(noFlyZones[i]);
        }
        
        var invalidEdges = new ArrayList<DefaultEdge>();

        // Iterate over all edges
        for (var edge : moveStationGraph.edgeSet()) {
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
            moveStationGraph.removeEdge(invalidEdge);
        }
    }
    
    // Convert a graph edge to Path2D object
    // Used to determine if a graph edge intersects a no fly zone
    private Path2D convertMoveStationGraphEdgeToPath2D(DefaultEdge edge) {
        Path2D edgePath = new Path2D.Double();

        var edgeSource = moveStationGraph.getEdgeSource(edge);
        var edgeTarget = moveStationGraph.getEdgeTarget(edge);
        
        edgePath.moveTo(edgeSource.longitude(), edgeSource.latitude());
        edgePath.lineTo(edgeTarget.longitude(),  edgeSource.latitude());
        // Must give some width to the path otherwise cannot compute intersection
        edgePath.lineTo(edgeTarget.longitude() + 0.0001, edgeTarget.latitude() + 0.0001);
        edgePath.lineTo(edgeSource.longitude() + 0.0001, edgeSource.latitude() + 0.0001);
        edgePath.lineTo(edgeSource.longitude(), edgeSource.latitude());
        edgePath.closePath();

        return edgePath;
    }
    
    // Adds move station as vertex in graph if valid
    public Point addVertex(Point moveStation) {
        
        if (moveStation == null || 
                checkMoveStationInNoFlyZone(moveStation) ||
                checkMoveStationOutsideCampusBoundary(moveStation)
                ) {
            return null;
        }

        moveStationGraph.addVertex(moveStation);
        return moveStation;
    }

    // Return true if move station within no fly zone
    private boolean checkMoveStationOutsideCampusBoundary(Point moveStation) {
        
        if (moveStation.latitude() >= App.BOUNDARY_LONG_LATS.get("maxLat") ||
                moveStation.latitude() <= App.BOUNDARY_LONG_LATS.get("minLat") ||
                moveStation.longitude() >= App.BOUNDARY_LONG_LATS.get("maxLong") ||
                moveStation.longitude() <= App.BOUNDARY_LONG_LATS.get("minLong")
                ) {
            return true;
        }
        
        return false;
    }
    
    // Return true if move station within no fly zone
    private boolean checkMoveStationInNoFlyZone(Point moveStation) {
        for (var noFlyZone : noFlyZones) {
            // Create Point2D object for inter-operability with Path2D contains().
            Point2D moveStation2D = new Point2D.Double(moveStation.longitude(), moveStation.latitude());

            if (noFlyZone.contains(moveStation2D)) {
                return true;
            }
        }
        return false;
    }
    
    public DefaultEdge addEdge(Point stationA, Point stationB) {
        try {
            var edge = moveStationGraph.addEdge(stationA, stationB);
    
            if (edge == null) {
                return null;
            }
            
            // Check edge does not intersect no fly zone
            for (var noFlyZoneArea : noFlyZoneAreas) {
                // Must convert to Path2D object in order to use intersect()
                var edgePath = convertMoveStationGraphEdgeToPath2D(edge);
                var edgeArea = new Area(edgePath);
    
                edgeArea.intersect(noFlyZoneArea);
    
                // If area is not empty then we know the edge overlaps no fly zone
                if (!edgeArea.isEmpty()) {
                    moveStationGraph.removeEdge(edge);
                    return null;
                }
            }
            return edge;
        } catch (IllegalArgumentException e) {
            // Stations may not exist in moveStationGraph
            return null;
        }
        
    }
    
    public boolean removeEdge(DefaultEdge edge) {
        return moveStationGraph.removeEdge(edge);
    }
    
    public boolean removeVertex(Point moveStation) {
        return moveStationGraph.removeVertex(moveStation);
    }
    
    private void addMoveStationGraphVertices(Point[][] moveStationGrid) {

//         Add each move station to the graph
        for (int i = 0; i < moveStationGrid.length; i++) {
            for (int j = 0; j < moveStationGrid[0].length; j++) {
                addVertex(moveStationGrid[i][j]);
            }
        }
        
        System.out.println("MSG CONTAINS DRONE START: " + moveStationGraph.containsVertex(droneStart.getLongLat()));
        // Add drone starting point as vertex TODO
        moveStationGraph.addVertex(droneStart.getLongLat());
    }

    // Returns 2D array of Points storing the coordinates of the move stations
    // that are distributed across campus in an equilateral pattern.
    private Point[][] createMoveStationsGrid() {

        // Longitude and latitude length for campus map boundary
        final Double longDistance = Utilities.absoluteDifference(
                App.BOUNDARY_LONG_LATS.get("maxLong"),
                App.BOUNDARY_LONG_LATS.get("minLong"));
        
        final Double latDistance = Utilities.absoluteDifference(
                App.BOUNDARY_LONG_LATS.get("maxLat"),
                App.BOUNDARY_LONG_LATS.get("minLat"));

        // Use trigonometry to compute distance between each row of move stations in the grid
        final double distanceBetweenRows = Math.sqrt(
                Math.pow(App.DRONE_MOVE_DISTANCE,2) -
                Math.pow(App.DRONE_MOVE_DISTANCE/2.0,2));

        // Compute shape for move stations matrix 
        final int moveStationColumns = (int) Math.floor(longDistance / App.DRONE_MOVE_DISTANCE) + 1;
        final int moveStationRows = (int) Math.floor(latDistance / distanceBetweenRows) + 1;
//        System.out.println("distance between rows is " + distanceBetweenRows); TODO
        // Used to ensure there is a move station with exact same coordinates as drone start position
        final Double startLong = computeStartLong(droneStart.getLongLat().longitude());
        final Double startLat = computeStartLat(droneStart.getLongLat().latitude(),distanceBetweenRows);
        
        var moveStationGrid = new Point[moveStationRows][moveStationColumns];
        
        this.offsetIndex = computeOffsetIndex(droneStart.getLongLat().latitude(),distanceBetweenRows);
        System.out.println(offsetIndex);
        // Calculate the coordinates of each move station
        for (int i = 0 ; i < moveStationRows ; i++) {
            
            // Offset longitude every offsetIndex-th row
            Double longOffset = ((i+offsetIndex) % 2) * App.DRONE_MOVE_DISTANCE/2.0;
            
            for (int j = 0 ; j < moveStationColumns; j++) {
                Double moveStationLong = startLong + (j * App.DRONE_MOVE_DISTANCE) + longOffset;
                
                Double moveStationLat = startLat - i * distanceBetweenRows;
                
                Point moveStation = Point.fromLngLat(moveStationLong, moveStationLat);
                moveStationGrid[i][j] = moveStation;
            }
        }

//       setInvalidMoveStationsToNull(moveStations); TODO delete
       return moveStationGrid;
    }
    
    // An offsetIndex of 1 means the first, third, fifth row ... are offset
    private int computeOffsetIndex(Double droneStartLat, Double distanceBetweenRows) {
        Double latDistanceFromBoundary = Math.abs(App.BOUNDARY_LONG_LATS.get("maxLat")-droneStartLat);
        int numDroneMovesFromBoundary = (int) Math.floor(latDistanceFromBoundary/distanceBetweenRows);
        return numDroneMovesFromBoundary % 2;
    }
    
    // Compute the longitude of the upper left move station in grid
    private Double computeStartLong(Double droneStartLong) {
        Double longDistanceFromBoundary = Math.abs(App.BOUNDARY_LONG_LATS.get("minLong")-droneStartLong);
        int numDroneMovesFromBoundary = (int) Math.floor(longDistanceFromBoundary/App.DRONE_MOVE_DISTANCE);
        return droneStartLong - App.DRONE_MOVE_DISTANCE * numDroneMovesFromBoundary;
    }
    
    // Compute the latitude of the upper left move station in grid
    private Double computeStartLat(Double droneStartLat, Double distanceBetweenRows) {
        Double latDistanceFromBoundary = Math.abs(App.BOUNDARY_LONG_LATS.get("maxLat")-droneStartLat);
        int numDroneMovesFromBoundary = (int) Math.floor(latDistanceFromBoundary/distanceBetweenRows);
        return droneStartLat + distanceBetweenRows * numDroneMovesFromBoundary;
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
    
    //  TODO delete
    //  needed for visualisation in report
    public ArrayList<Feature> moveStationsGraphToGeojson(String filename) {
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
        Utilities.writeFile(filename+".geojson", geojson);
          
        return featuresList;
    }
}
