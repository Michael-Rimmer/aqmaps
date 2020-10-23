package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;

import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultEdge;

import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

public class DroneMove {
    
    private final Sensor sourceSensor;
    private final Sensor targetSensor;
    
    
    public DroneMove(Sensor sourceSensor, Sensor targetSensor, GraphPath<Point, DefaultEdge> moveStationsPath) {
        this.sourceSensor = sourceSensor;
        this.targetSensor = targetSensor;
    }
}
