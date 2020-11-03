package uk.ac.ed.inf.aqmaps;

import java.util.HashMap;

/**
 * Hello world!
 *
 */
public class App 
{
    // Declare constants
    public static final double MAX_DRONE_MOVE_DISTANCE = 0.0003;
    
    // The precise meaning of close to in coursework specs
    public static final double CLOSE_ENOUGH_DISTANCE = 0.0003;
    
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
        Utilities.validateArgs(args);
        
//        String day = args[0];
//        String month = args[1];
//        String year = args[2];
        final String lat = args[3];
        final String lng = args[4];
        final String httpPort = args[6];
        
        final var droneStart = Utilities.createDroneStartPoint(lng, lat);
        
        final HttpClientWrapper clientWrapper = new HttpClientWrapper(httpPort);
        
        var noFlyZones = clientWrapper.getNoFlyZones();

//        String[] years = {"2020", "2021"};
        String[] months = {"01", "02", "03", "04", "05", "06","07","08","09","10","11","12"};
//        String[] days = {"01","02","03","04","05","06","07","08","09","10","11","12","13","14","15","16","17","18","19","20","21","22","23","24","25","26","27","28","29","30","31"};
        
        String[] years = {"2020"};
//        String[] months = {"01"};
        String[] days = {"01"};
        
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
                        System.out.println("ERROR: Failed to fly drone on day: " + String.format("%s-%s-%s.", day,month,year));
                        System.out.println(e);
                        break;
                    }
                }
            }
        }

    }

}
