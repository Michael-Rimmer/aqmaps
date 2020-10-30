package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Point;

// Class to represent a location that the drone must visit during its flight path
public class MustVisitLocation {
    private Point closestMoveStation;
    private Point longLat;
    private boolean visited = false;

    // Empty constructor
    public MustVisitLocation() {}

    public MustVisitLocation(Point longLat) {
        this.longLat = longLat;
    }
    
    public void setLongLat(double longitude, double latitude) {
        this.longLat = Point.fromLngLat(longitude, latitude);
    }
    
    public Point getLongLat() {
        return this.longLat;
    }
    
    public void setVisited(boolean visited) {
        this.visited = visited;
    }
    
    public boolean getVisited() {
        return visited;
    }
    
    public void setClosestMoveStation(Point moveStation) {
        if(!closestStationIsValid(moveStation)) {
            System.out.println(
                    "WARNING: Invalid closest move station for MustVisitLocation with coords: " + getLongLat());
        }

        // Regardless, assign closest move station because do not want to fail entire 
        // app on basis of one move station
        this.closestMoveStation = moveStation;
    }
  
    public Point getClosestMoveStation() {
        return this.closestMoveStation;
    }
    
    // Check closest move station is no further than from this objects coordinates
    private boolean closestStationIsValid(Point moveStation) {

        double distance = Utilities.euclideanDistance(getLongLat(), moveStation);

        if (distance < App.MAX_DRONE_MOVE_DISTANCE) {
            return true;
        }

        return false;
    }
}
