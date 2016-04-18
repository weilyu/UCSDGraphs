package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by lvwei on 2016/04/17.
 */
public class MapNode implements Comparable<MapNode> {
    private GeographicPoint location;
    private List<MapEdge> edges;
    private List<MapNode> neighbors;
    private double distance;
    private double actDis;

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new ArrayList<>();
        neighbors = new ArrayList<>();
        distance = Double.MAX_VALUE;
    }

    public int countEdges() {
        return edges.size();
    }

    public boolean addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
        if (from == null || to == null || roadName == null || roadType == null || length <= 0) return false;
        MapEdge toAdd = new MapEdge(from, to, roadName, roadType, length);
        for (MapEdge me : edges) {
            if (me.equals(toAdd)) return false;
        }
        edges.add(toAdd);
        return true;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public List<MapNode> getNeighbors() {
        return neighbors;
    }

    public void addNeighbor(MapNode neighbor) {
        neighbors.add(neighbor);
    }

    public double getDistanceTo(MapNode to) {
        if (!neighbors.contains(to)) throw new IllegalArgumentException("Cannot find path");
        GeographicPoint toLocation = to.getLocation();
        for (MapEdge me : edges) {
            if (me.getTo().equals(toLocation)) {
                return me.getLength();
            }
        }
        return 0;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public int compareTo(MapNode o) {
        return Double.compare(distance, o.getDistance());
    }

    public double getActDis() {
        return actDis;
    }

    public void setActDis(double actDis) {
        this.actDis = actDis;
    }
}
