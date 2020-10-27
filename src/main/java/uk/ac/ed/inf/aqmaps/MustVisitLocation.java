package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Point;

public class MustVisitLocation {
    
    private Point closestMoveStation;
    protected Point longLat;
    protected boolean visited = false;
    
    public MustVisitLocation() {
    }
    
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
        this.closestMoveStation = moveStation;
    }
  
    private double euclideanDistance(Point a, Point b) {
      // Computes Euclidean distance between two geojson point objects
        return Math.sqrt(Math.pow(a.longitude()-b.longitude(),2) + Math.pow(a.latitude()-b.latitude(),2));
    }
  
    public Point getClosestMoveStation() {
        return this.closestMoveStation;
    }
}
