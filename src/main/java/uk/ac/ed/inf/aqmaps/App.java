package uk.ac.ed.inf.aqmaps;

import java.util.HashMap;
import java.util.stream.IntStream;

public class App 
{
    // Declare constants
    public static final Double DRONE_MOVE_DISTANCE = 0.0003;

    // Maximum acceptable distance between drone start and end positions
    public static final Double CLOSE_TO_STARTING_POINT_DISTANCE = 0.0003;
    
    // Max distance drone can be from sensor to download its readings
    public static final Double DRONE_RECEIVER_MAX_DISTANCE = 0.0002;
    
    // Stores the angles that drone can fly in measured in degrees
    public static final int[] DRONE_DIRECTION_ANGLES = IntStream.rangeClosed(0, 35)
            .map(i -> i*10)
            .toArray();

    @SuppressWarnings("serial")
    public static final HashMap<String,Double> BOUNDARY_LONG_LATS = new HashMap<String,Double>() {{
        put("minLong", -3.192473);
        put("minLat", 55.942617);
        put("maxLong", -3.184319);
        put("maxLat", 55.946233);
     }};

    public static void main( String[] args ) throws Exception
    {
            System.out.println("-----------------------------------------------");
            
            // Parse command line args
//            Utilities.validateArgs(args);
//            
//            final String day = args[0];
//            final String month = args[1];
//            final String year = args[2];
//            final String lat = args[3];
//            final String lng = args[4];
//            final String httpPort = args[6];
//            String[] days = {"01","02","03","04","05","06","07","08","09","10","11","12"};
//            String[] days = {"02"};
//            String[] years = {"2020", "2021"};
//            String[] months = {"01", "02", "03", "04", "05", "06","07","08","09","10","11","12"};
//            String[] days = {"01","02","03","04","05","06","07","08","09","10","11","12","13","14","15","16","17","18","19","20","21","22","23","24","25","26","27","28","29","30","31"};
            String[] years = {"2020"};
            String[] months = {"03"};
            String[] days = {"27"};
            
            String lng = "-3.188396"; // TODO
            String lat = String.valueOf(55.944425);
            String httpPort = "80";
            
            // Create drone start location
            final var droneStart = new MustVisitLocation(lng, lat);
            
            // Create Http Wrapper to request data from server
            final HttpClientWrapper clientWrapper = new HttpClientWrapper(httpPort);
            
            // Get no fly zones
            var noFlyZones = clientWrapper.getNoFlyZones();
            
            for (String year : years) {
                for (String month : months) {
                    for (String day : days) {
            
                    try {
        
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
                      e.printStackTrace();
                      System.err.println("ERROR: Failed to fly drone:\n" + e.getMessage());
//                      System.exit(1);
                  }
                }
            }
//            System.out.println("-----------------------------------------------");
        
        }
    }

}
