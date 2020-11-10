package uk.ac.ed.inf.aqmaps;

import java.util.HashMap;

public class App 
{
    // Declare constants
    public static final Double MAX_DRONE_MOVE_DISTANCE = 0.0003;

    // The precise meaning of close to in coursework specs
    public static final Double CLOSE_ENOUGH_DISTANCE = 0.0003;

    @SuppressWarnings("serial")
    public static final HashMap<String,Double> BOUNDARY_LONG_LATS = new HashMap<String,Double>() {{
        put("minLong", -3.192473);
        put("minLat", 55.942617);
        put("maxLong", -3.184319);
        put("maxLat", 55.946233);
     }};

    public static void main( String[] args ) throws Exception
    {
        try {
            System.out.println("-----------------------------------------------");
            
            // Parse command line args
            Utilities.validateArgs(args);
            
            final String day = args[0];
            final String month = args[1];
            final String year = args[2];
            final String lat = args[3];
            final String lng = args[4];
            final String httpPort = args[6];
            
            // Create drone start location
            final var droneStart = new MustVisitLocation(lng, lat);
            
            // Create Http Wrapper to request data from server
            final HttpClientWrapper clientWrapper = new HttpClientWrapper(httpPort);
            
            // Get no fly zones
            var noFlyZones = clientWrapper.getNoFlyZones();

            // Get air quality readings for given date
            var sensors = clientWrapper.getAirQualityData(year, month, day);

            // Compute drone flight path
            var dpf = new DronePathFinder(droneStart, sensors, noFlyZones);
            var droneMoves = dpf.getDroneMoves();
            
            // Instantiate drone with movement instructions
            Drone drone = new Drone(droneMoves, day, month, year);
            
            // Generate output
            drone.generateReadingsGeojson();
            drone.generateFlightPath();

            System.out.println("-----------------------------------------------");
        } catch (Exception e) {
            System.err.println("ERROR: Failed to fly drone:\n" + e.getMessage());
            System.exit(1);
        }

    }

}
