package uk.ac.ed.inf.aqmaps;

import static org.junit.Assert.assertTrue;

import java.awt.geom.Path2D;
import java.util.ArrayList;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.junit.Before;
import org.junit.Test;

import com.mapbox.geojson.Point;

public class DronePathFinderTest {
    
    private static boolean setUpComplete = false;

    // Mock command line arguments
    private static String day = "01";
    private static String month = "01";
    private static String year = "2020";
    private static String droneStartLat = "55.9444";
    private static String droneStartLong = "-3.1878";
    private static String httpPort = "80";
    
    private static MustVisitLocation droneStart;
    private static HttpClientWrapper clientWrapper;
    private static Path2D[] noFlyZones;
    private static ArrayList<Sensor> sensors;
    private static DronePathFinder dpf;
    
    
    @Before
    public void setUp() throws Exception {
        if (setUpComplete) {
            return;
        }
        
        droneStart = Utilities.createDroneStartPoint(droneStartLong, droneStartLat);
        clientWrapper = new HttpClientWrapper(httpPort);
        noFlyZones = clientWrapper.getNoFlyZones();
        sensors = clientWrapper.getAirQualityData(year, month, day);
        dpf = new DronePathFinder(droneStart, sensors, noFlyZones, App.BOUNDARY_LONG_LATS, App.MAX_DRONE_MOVE_DISTANCE);

        setUpComplete = true;
    }
    
    @Test
    public void testAllMoveStationLatsWithinBoundary()
    {
        // maximum and minimum lat values for campus boundary
        final Double minLat = App.BOUNDARY_LONG_LATS[1];
        final Double maxLat = App.BOUNDARY_LONG_LATS[3];

        var moveStationsGraph = dpf.getMoveStationsGraph();
        for (var vertex : moveStationsGraph.vertexSet()) {
            // must be strictly inside the boundary
            assertTrue( vertex.latitude() > minLat && vertex.latitude() < maxLat);
        }
    }
    
    @Test
    public void testAllMoveStationLongsWithinBoundary()
    {
        // maximum and minimum long values for campus boundary
        final Double minLong = App.BOUNDARY_LONG_LATS[0];
        final Double maxLong = App.BOUNDARY_LONG_LATS[2];
        
        var moveStationsGraph = dpf.getMoveStationsGraph();
        for (var vertex : moveStationsGraph.vertexSet()) {
            // must be strictly inside the boundary
            assertTrue( vertex.longitude() > minLong && vertex.longitude() < maxLong);
        }
    }
}
