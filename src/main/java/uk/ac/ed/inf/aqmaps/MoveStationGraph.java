package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.Graphs;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultUndirectedGraph;

import com.mapbox.geojson.Point;

public class MoveStationGraph {

    private final Path2D[] noFlyZones;
    
    private final MustVisitLocation droneStart;

    // Graph to store move stations and the paths between them
    private Graph<Point,DefaultEdge> moveStationGraph = 
            new DefaultUndirectedGraph<>(DefaultEdge.class);

    public MoveStationGraph(Path2D[] noFlyZones, MustVisitLocation droneStart) {
        this.noFlyZones = noFlyZones;
        this.droneStart = droneStart;
        createMoveStationGraph();
    }
    
    public GraphPath<Point, DefaultEdge> computeShortestPathBetweenMoveStations(Point a, Point b) {
        // Use Dijkstras algorithm
        // Note this is a greedy algorithm therefore may produce different result each time
        var shortestPathFinder = new DijkstraShortestPath<Point, DefaultEdge>(moveStationGraph);
        var shortestPath = shortestPathFinder.getPath(a, b);
        return shortestPath;
    }
    
    // Returns a move station that is connected by an edge to the given move station
    public Point getNeighbourMoveStation(Point station) {
        return Graphs.neighborListOf(moveStationGraph, station).get(0);
    }
    
    // Return a set of the vertices in the graph
    public Set<Point> getVertexSet() {
        return moveStationGraph.vertexSet();
    }

    // Pre-computing the points (move stations) on the map that the drone may move to.
    // Move stations are distributed across campus map in an equilateral triangle pattern.
    // This means drone is constrained to move in directions that are multiples of 60 degrees: 0, 60, 120, 240, 300
    private void createMoveStationGraph() {
        Point[][] moveStationGrid = createMoveStationsGrid();
        addMoveStationGraphVertices(moveStationGrid);
        addMoveStationsGraphEdges(moveStationGrid);
        // Remove any move stations that have no edges
        removeIsolatedMoveStations();
    }

    // Remove any move stations that have no edges from the graph
    private void removeIsolatedMoveStations() {
       
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
    
    // Adds edges between droneStart move station and other adjacent move stations
    private void connectDroneStartToMoveStations(Point[][] moveStations) {
        for (Point moveStation : moveStationGraph.vertexSet()) {
            if (Utilities.euclideanDistance(moveStation, droneStart.getLongLat()) < App.MAX_DRONE_MOVE_DISTANCE) {
                moveStationGraph.addEdge(droneStart.getLongLat(), moveStation);
            }
        }
    }
    
    // Adds edges in the graph to form paths between move stations
    private void addMoveStationsGraphEdges(Point[][] moveStationGrid) {

        // Connect move stations in the graid
        for (int i = 0; i < moveStationGrid.length; i++) {
            for (int j = 0; j < moveStationGrid[0].length - 1; j++) {

                // Ensure not adding edge to an invalid move station
                // Connect every move station to its rightmost neighbour
                if (moveStationGrid[i][j] != null && moveStationGrid[i][j+1] != null) {
                    moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i][j+1]);
                }
                
                // For every second row, connect move stations to the closest two above and closest two below.
                if (i % 2 == 1 && moveStationGrid[i][j] != null) {
                    if (moveStationGrid[i-1][j] != null) moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i-1][j]);
                    if (moveStationGrid[i-1][j+1] != null) moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i-1][j+1]);
                    // Only connect move station to closest two below if not on last row
                    if (i != moveStationGrid.length -1) {
                        if (moveStationGrid[i+1][j] != null) moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i+1][j]);
                        if (moveStationGrid[i+1][j+1] != null) moveStationGraph.addEdge(moveStationGrid[i][j], moveStationGrid[i+1][j+1]);
                    }
                }
            }
        }

        // Corner case since droneStart is not in moveStationGrid
        connectDroneStartToMoveStations(moveStationGrid);
        removeInvalidMoveStationEdges();
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
        moveStationGraph.addVertex(droneStart.getLongLat());
        
        return moveStationGrid;
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
}
