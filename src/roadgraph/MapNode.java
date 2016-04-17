package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by lvwei on 2016/04/17.
 */
public class MapNode {
    private GeographicPoint location;
    private List<MapEdge> edges;
    private List<MapNode> neighbors;

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new ArrayList<>();
        neighbors = new ArrayList<>();
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
}
