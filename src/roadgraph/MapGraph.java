/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 *         <p>
 *         A class which represents a graph of geographic locations
 *         Nodes in the graph are intersections between
 */
public class MapGraph {
    private HashMap<GeographicPoint, MapNode> nodeMap;


    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        nodeMap = new HashMap<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return nodeMap.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return nodeMap.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        int count = 0;
        for (MapNode mn : nodeMap.values()) {
            count += mn.countEdges();
        }
        return count;
    }


    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (location == null) return false;
        for (GeographicPoint gp : nodeMap.keySet()) {
            if (gp.equals(location)) return false;
        }
        nodeMap.put(location, new MapNode(location));
        return true;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {
        MapNode curNode = nodeMap.get(from);
        if (!curNode.addEdge(from, to, roadName, roadType, length)) {
            throw new IllegalArgumentException("Wrong input");
        } else curNode.addNeighbor(nodeMap.get(to));
    }


    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        Queue<MapNode> queue = new LinkedList<>();
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode, MapNode> parent = new HashMap<>();

        MapNode startNode = nodeMap.get(start);
        MapNode goalNode = nodeMap.get(goal);
        queue.add(startNode);
        visited.add(startNode);

        while (queue.size() > 0) {
            MapNode curNode = queue.poll();
            nodeSearched.accept(curNode.getLocation());
            if (curNode.equals(goalNode)) return getPath(startNode, goalNode, parent);

            List<MapNode> neighbors = curNode.getNeighbors();
            for (MapNode neighbor : neighbors) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    parent.put(neighbor, curNode);
                    queue.add(neighbor);
                }
            }
        }

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    private List<GeographicPoint> getPath(MapNode startNode, MapNode goalNode, HashMap<MapNode, MapNode> parent) {
        ArrayList<GeographicPoint> path = new ArrayList<>();
        MapNode curNode = goalNode;
        while (true) {
            if (curNode.equals(startNode)) {
                path.add(curNode.getLocation());
                break;
            }
            path.add(curNode.getLocation());
            curNode = parent.get(curNode);
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        MapNode startNode = nodeMap.get(start);
        MapNode goalNode = nodeMap.get(goal);
        if (startNode == null || goalNode == null) {
            throw new NullPointerException("Null start/goal");
        }

        //initialize
        PriorityQueue<MapNode> pq = new PriorityQueue<>();
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode, MapNode> parent = new HashMap<>();
        for (MapNode mn : nodeMap.values()) {
            mn.setDistance(Double.MAX_VALUE);
        }
        int count = 0;

        startNode.setDistance(0);
        pq.add(startNode);

        while (!pq.isEmpty()) {
            MapNode curr = pq.poll();
            count++;
            if (!visited.contains(curr)) {
                visited.add(curr);
                nodeSearched.accept(curr.getLocation());
                if (curr == goalNode){
                    System.out.println(count);
                    return getPath(startNode, goalNode, parent);
                }
                for (MapNode n : curr.getNeighbors()) {
                    if (!visited.contains(n)) {
                        double pathToN = curr.getDistance() + curr.getDistanceTo(n);
                        if (pathToN < n.getDistance()) {
                            n.setDistance(pathToN);
                            parent.put(n, curr);
                            pq.add(n);
                        }
                    }
                }
            }
        }
        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        MapNode startNode = nodeMap.get(start);
        MapNode goalNode = nodeMap.get(goal);
        if (startNode == null || goalNode == null) {
            throw new NullPointerException("Null start/goal");
        }

        //initialize
        PriorityQueue<MapNode> pq = new PriorityQueue<>();
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode, MapNode> parent = new HashMap<>();
        for (MapNode mn : nodeMap.values()) {
            mn.setDistance(Double.MAX_VALUE);
        }
        int count = 0;
        pq.add(startNode);

        while (!pq.isEmpty()) {
            MapNode curr = pq.poll();
            count++;
            if (!visited.contains(curr)) {
                visited.add(curr);
                nodeSearched.accept(curr.getLocation());
                if (curr == goalNode){
                    System.out.println(count);
                    return getPath(startNode, goalNode, parent);
                }
                for (MapNode n : curr.getNeighbors()) {
                    if (!visited.contains(n)) {
                        GeographicPoint nLoc = n.getLocation();
                        double distanceN = nLoc.distance(start) + nLoc.distance(goal);
                        if (distanceN < n.getDistance()) {
                            n.setDistance(distanceN);
                            parent.put(n, curr);
                            pq.add(n);
                        }
                    }
                }
            }
        }
        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }


    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");

        // You can use this method for testing.


        MapGraph theMap1 = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap1);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap1.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap1.aStarSearch(start,end);



    }

}
