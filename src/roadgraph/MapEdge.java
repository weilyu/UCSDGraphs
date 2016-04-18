package roadgraph;

import geography.GeographicPoint;

/**
 * Created by lvwei on 2016/04/17.
 */
public class MapEdge {
    private GeographicPoint from;
    private GeographicPoint to = null;
    private String roadName;
    private String roadType;
    private double length;

    public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public double getLength() {
        return length;
    }
}
