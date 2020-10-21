package uk.ac.ed.inf.aqmaps;

import static org.junit.Assert.assertTrue;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.junit.Test;

import com.mapbox.geojson.Point;


/**
 * Unit test for simple App.
 */
public class AppTest 
{
    @Test
    public void testAllMoveStationLatsWithinBoundary()
    {
        // boundaryLongLats elements in format: minLong, minLat, maxLong, maxLat
        final Double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};

        // maximum and minimum lat and long values for campus boundary
        final Double minLat = BOUNDARY_LONG_LATS[1];
        final Double maxLat = BOUNDARY_LONG_LATS[3];

        Point[][] moveStations = App.createMoveStationVertices();
        for (int i = 0; i<moveStations.length;i++) {
            for (int j = 0; j<moveStations[0].length;j++) {
                assertTrue( moveStations[i][j].latitude() >= minLat && moveStations[i][j].latitude() <= maxLat);
            }
        }
    }
    
    @Test
    public void testAllMoveStationLongsWithinBoundary()
    {
        // boundaryLongLats elements in format: minLong, minLat, maxLong, maxLat
        final Double[] BOUNDARY_LONG_LATS = {-3.192473, 55.942617, -3.184319, 55.946233};

        // maximum and minimum lat and long values for campus boundary
        final Double minLong = BOUNDARY_LONG_LATS[0];
        final Double maxLong = BOUNDARY_LONG_LATS[2];

        Point[][] moveStations = DronePathFinder.createMoveStationVertices();
        for (int i = 0; i<moveStations.length;i++) {
            for (int j = 0; j<moveStations[0].length;j++) {
                assertTrue( moveStations[i][j].longitude() >= minLong && moveStations[i][j].longitude() <= maxLong);
            }
        }
    }
    
    @Test
    public void testAllVerticesAddedToGraph()
    {
        Graph<Point,DefaultEdge> campusGraph = App.createMoveStationGraph();
        
        Point[][] moveStationVertices = App.createMoveStationVertices();
        
        assertTrue( campusGraph.vertexSet().size() == moveStationVertices.length * moveStationVertices[0].length);
        
    }
}
