package uk.ac.ed.inf.aqmaps;

import static org.junit.Assert.assertTrue;

import java.awt.geom.Path2D;
import java.util.ArrayList;

import org.junit.Before;
import org.junit.Test;

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
        dpf = new DronePathFinder(droneStart, sensors, noFlyZones);

        setUpComplete = true;
    }
    
    @Test
    public void testAllMoveStationLatsWithinBoundary()
    {
        // maximum and minimum lat values for campus boundary
        final Double minLat = App.BOUNDARY_LONG_LATS.get("minLat");
        final Double maxLat = App.BOUNDARY_LONG_LATS.get("maxLat");

        var moveStationsGraph = dpf.getMoveStationGraph();
        for (var vertex : moveStationsGraph.vertexSet()) {
            // must be strictly inside the boundary
            assertTrue( vertex.latitude() > minLat && vertex.latitude() < maxLat);
        }
    }
    
    @Test
    public void testAllMoveStationLongsWithinBoundary()
    {
        // maximum and minimum long values for campus boundary
        final Double minLong = App.BOUNDARY_LONG_LATS.get("minLong");
        final Double maxLong = App.BOUNDARY_LONG_LATS.get("maxLong");
        
        var moveStationsGraph = dpf.getMoveStationGraph();
        for (var vertex : moveStationsGraph.vertexSet()) {
            // must be strictly inside the boundary
            assertTrue( vertex.longitude() > minLong && vertex.longitude() < maxLong);
        }
    }
}
