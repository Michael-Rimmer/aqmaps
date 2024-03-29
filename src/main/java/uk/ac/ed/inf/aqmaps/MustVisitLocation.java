package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Point;

// Class to represent a location that the drone must visit during its flight path
public class MustVisitLocation {
    protected Point closestMoveStation; // TODO refactor with sensor inheritance
    private Point longLat;
    private boolean visited = false;

    // Empty constructor necessary because Sensor subclass does
    // not have access to longLat values at time of initialisation
    public MustVisitLocation() {}

    public MustVisitLocation(String lngString, String latString) {
        final Double lat = Double.parseDouble(latString);
        final Double lng = Double.parseDouble(lngString);
        this.longLat = Point.fromLngLat(lng, lat);
    }
    
    public void setLongLat(Double longitude, Double latitude) {
        longLat = Point.fromLngLat(longitude, latitude);
    }
    
    public Point getLongLat() {
        return longLat;
    }
    
    public void setVisited(boolean visited) {
        this.visited = visited;
    }
    
    public boolean getVisited() {
        return visited;
    }
    
    public void setClosestMoveStation(Point moveStation) { //TODO REFACTOR WITH SENSOR OVERRIDE
        // Validate
        if (!closestStationIsValid(moveStation)) {
            System.out.println("--------");
            System.out.println("WARNING: WARNING:WARNING:WARNING:WARNING:WARNING:WARNING:WARNING:WARNING:WARNING:WARNING:");
            System.out.println("WARNING: Invalid closest move station for MustVisitLocation.");
            System.out.println("MustVisitLocation coords: " + getLongLat());
            System.out.println("Move station coords: " + moveStation.coordinates());
            System.out.println("--------");
        }

        // Regardless, assign closest move station because do not want to fail entire 
        // app on basis of one move station
        closestMoveStation = moveStation;
    }
  
    public Point getClosestMoveStation() {
        return closestMoveStation;
    }
    
    public boolean closestStationIsValid(Point moveStation) {

        double distance = Utilities.euclideanDistance(getLongLat(), moveStation);

        // Check closest move station is no further than App.CLOSE_ENOUGH_DISTANCE from this object's coordinates
        if (distance < App.DRONE_SENSOR_RANGE) {
            return true;
        } 

        return false;
    }
}
