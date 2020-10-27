package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

public class GeojsonUtilities {
    
    public static Feature createLineStringFeature(Point a, Point b) {
        var lineLongLats = new ArrayList<Point>(2);
        lineLongLats.add(a);
        lineLongLats.add(b);
        LineString lineString = LineString.fromLngLats(lineLongLats);
        Feature lineFeature = Feature.fromGeometry(lineString);
        return lineFeature;
    }
    
}
