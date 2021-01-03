import java.util.*;

public class GraphOperations {

    public static class Edge implements Comparable<Edge> {

        int u;
        int v;
        int weight;

        public Edge(int u, int v, int weight) {
            this.u = u;
            this.v = v;
            this.weight = weight;
        }

        @Override
        public int compareTo(Edge o) {
            return this.weight - o.weight;
        }

        @Override
        public String toString() {
            return "Edge{" +
                    "u=" + u +
                    ", v=" + v +
                    ", weight=" + weight +
                    '}';
        }
    }

    public static class Graph {
        int V;
        int[][] graph;
        List<Edge> edges;
        boolean isDirected;

        public Graph(int V, boolean isDirected) {
            this.V = V;
            this.isDirected = isDirected;
            graph = new int[V][V];
            edges = new ArrayList<>();
        }

        public void addEdge(int u, int v, int weight) {
            if (isDirected) {
                graph[u][v] = weight;
                edges.add(new Edge(u, v, weight));
            } else {
                graph[u][v] = graph[v][u] = weight;
                edges.add(new Edge(u, v, weight));
                edges.add(new Edge(v, u, weight));
            }
        }

        public void topSort() {
            List<Integer> visited = new ArrayList<>();
            LinkedList<Integer> sortedList = new LinkedList<>();
            StringBuilder sb = new StringBuilder();

            visited.add(0);
            topSortUtil(0, visited, sortedList);

            sb.append("\n");
            sb.append("----------- TOPOLOGICAL-SORT -----------");
            sb.append("\n");

            for (int v : sortedList) {
                sb.append(v).append(", ");
            }
            System.out.println(sb.toString());

        }

        private void topSortUtil(int u, List<Integer> visited, LinkedList<Integer> sortedList) {
            for (int v = 0; v < V; v++) {
                if (graph[u][v] != 0 && !visited.contains(v)) {
                    visited.add(v);
                    topSortUtil(v, visited, sortedList);
                }
            }
            sortedList.add(u);
        }

        public void dfs(int s) {
            List<Integer> visited = new ArrayList<>();
            StringBuilder sb = new StringBuilder();

            visited.add(s);
            dfsUtil(s, visited);

            sb.append("\n");
            sb.append("----------- DEPTH-FIRST-SEARCH -----------");
            sb.append("\n");

            for (int v : visited) {
                sb.append(v).append(" -> ");
            }
            System.out.println(sb.toString());
        }

        private void dfsUtil(int u, List<Integer> visited) {
            for (int v = 0; v < V; v++) {
                if (graph[u][v] != 0 && !visited.contains(v)) {
                    visited.add(v);
                    dfsUtil(v, visited);
                }
            }
        }

        public void bfs(int s) {
            boolean[] visited = new boolean[V];
            Queue<Integer> Q = new LinkedList<>();
            StringBuilder sb = new StringBuilder();

            Q.add(s);
            visited[s] = true;

            sb.append("\n");
            sb.append("----------- BREADTH-FIRST-SEARCH -----------");
            sb.append("\n");

            while (!Q.isEmpty()) {
                int u = Q.remove();
                sb.append(u).append(" -> ");

                for (int v = 0; v < V; v++) {
                    if (graph[u][v] != 0 && !visited[v]) {
                        visited[v] = true;
                        Q.add(v);
                    }
                }
            }

            System.out.println(sb.toString());
        }

        public void dijkstra(int s) {

            int[] dist = new int[V];
            int[] parent = new int[V];
            int[] Q = new int[V];

            Arrays.fill(dist, Integer.MAX_VALUE);

            dist[s] = 0;

            for (int count = 0; count < V - 1; count++) {
                int u = minKey(dist, Q);
                Q[u] = 1;
                for (int v = 0; v < V; v++) {
                    if (graph[u][v] != 0 && Q[v] == 0 && dist[v] > dist[u] + graph[u][v]) {
                        dist[v] = dist[u] + graph[u][v];
                        parent[v] = u;
                    }
                }
            }

            StringBuilder sb = new StringBuilder();

            sb.append("\n");
            sb.append("----------- DIJKSTRA -----------");
            sb.append("\n");

            for (int i = 0; i < dist.length; i++) {
                sb.append(s + " -> " + i + " distance: " + dist[i]).append("\n");
            }

            System.out.println(sb.toString());
        }

        public boolean bf(int s) {
            StringBuilder sb = new StringBuilder();

            int parent[] = new int[V];
            int dist[] = new int[V];

            // init
            Arrays.fill(dist, Integer.MAX_VALUE);
            dist[s] = 0;

            // relax
            for (int i = 0; i < V - 1; i++) {
                for (Edge edge : edges) {
                    int u = edge.u;
                    int v = edge.v;
                    int w = edge.weight;
                    if (dist[u] != Integer.MAX_VALUE && dist[v] > dist[u] + w) {
                        dist[v] = dist[u] + w;
                        parent[v] = u;
                    }
                }
            }

            // detect negative cycle
            for (Edge edge : edges) {
                if (dist[edge.v] > dist[edge.v] + edge.weight) {
                    return false;
                }
            }

            sb.append("\n");
            sb.append("----------- BELLMAN-FORD -----------");
            sb.append("\n");

            for (int i = 0; i < dist.length; i++) {
                sb.append(s + " -> " + i + " distance: " + dist[i]).append("\n");
            }

            System.out.println(sb.toString());
            return true;
        }


        public void kruskal() {
            int[][] mst = new int[graph.length][graph.length];

            Collections.sort(edges);

            /*System.out.println();
            for (Edge e : edges) {
                System.out.println(e + " ");
            }*/

            int[] parents = new int[V];
            int[] size = new int[V];

            // initialize parents and size
            // all the vertices are a set in themselves
            // they are the parent of their own set
            // the size of their set is also 1
            for (int i = 0; i < graph.length; i++) {
                parents[i] = i;
                size[i] = 1;
            }

            int counter = 0;
            int edgeTaken = 1;
            // connecting all vertices
            // we need at least V - 1 edges
            while (edgeTaken <= V - 1) {
                Edge e = edges.get(counter);
                counter++;

                if (hasCycle(e.u, e.v, parents)) {
                    continue;
                }

                // for combining the edge into a MST,
                // we first need to find the parents of both vertices
                int u = findParent(e.u, parents);
                int v = findParent(e.v, parents);

                // print parents
                /*System.out.println();
                for (int p : parents) {
                    System.out.print(p + " ");
                }*/

                union(u, v, parents, size);

                // print parents
                /*System.out.println();
                for (int p : parents) {
                    System.out.print(p + " ");
                }*/

                mst[e.u][e.v] = e.weight;
                edgeTaken++;

            }
            printMSTKruskal(mst);
        }

        private void printMSTKruskal(int[][] mst) {
            System.out.println("----------- KRUSKAL -----------");
            for (int i = 0; i < mst.length; i++) {
                for (int j = 0; j < mst.length; j++) {
                    if (mst[i][j] != 0) {
                        System.out.println(i + " -> " + j + " w: " +
                                +mst[i][j]);
                    }
                }
            }
        }

        private void union(int u, int v, int[] parents, int[] size) {
            // find the parent of both the vertices in the current
            // edge, and merge the larger disjoint set with the smaller set

            u = findParent(u, parents);
            v = findParent(v, parents);

            if (size[u] > size[v]) {
                parents[v] = u;
                size[u] += size[v];
            } else {
                parents[u] = v;
                size[v] += size[u];
            }

        }

        private boolean hasCycle(int u, int v, int[] parents) {
            // if the parent of the set of both vertices are same,
            // this means they are connected to a common vertex
            // if we put it in the MST, it will create a cycle
            return findParent(u, parents) == findParent(v, parents);
        }

        private int findParent(int u, int[] parents) {
            // if the parent of the set of any vertex is the vertex itself,
            // then this is the parent of the vertex of the current
            // edge being processed
            if (parents[u] == u) {
                return u;
            } else {
                // parent compression
                parents[u] = findParent(parents[u], parents);
                return parents[u];
            }
        }

        public void prim() {
            int parent[] = new int[V];
            int key[] = new int[V];
            int Q[] = new int[V];

            for (int i = 0; i < V; i++) {
                key[i] = Integer.MAX_VALUE;
                Q[i] = 0;
            }

            key[0] = 0;
            parent[0] = -1;

            for (int count = 0; count < V - 1; count++) {
                int u = minKey(key, Q);
                Q[u] = 1;

                for (int v = 0; v < V; v++) {
                    if (graph[u][v] != 0 && Q[v] == 0 && graph[u][v] < key[v]) {
                        parent[v] = u;
                        key[v] = graph[u][v];
                    }
                }
            }

            printMSTPrim(parent);
        }

        private void printMSTPrim(int[] parent) {
            System.out.println("----------- PRIM -----------");

            for (int i = 1; i < V; i++)
                System.out.println(parent[i] + " -> " + i + "\t" + graph[i][parent[i]]);
        }

        private int minKey(int[] key, int[] Q) {
            int min = Integer.MAX_VALUE;
            int minIndex = -1;
            for (int i = 0; i < V; i++) {
                if (Q[i] == 0 && key[i] < min) {
                    min = key[i];
                    minIndex = i;
                }
            }
            return minIndex;
        }
    }

    public static void main(String[] args) {
        Graph graph = new Graph(9,false);
        graph.addEdge(0, 1, 4);
        graph.addEdge(1, 2, 8);
        graph.addEdge(2, 3, 7);
        graph.addEdge(3, 4, 9);
        graph.addEdge(4, 5, 10);
        graph.addEdge(5, 6, 2);
        graph.addEdge(6, 7, 1);
        graph.addEdge(7, 0, 8);
        graph.addEdge(1, 7, 11);
        graph.addEdge(7, 8, 7);
        graph.addEdge(6, 8, 6);
        graph.addEdge(2, 8, 2);
        graph.addEdge(2, 5, 4);
        graph.addEdge(3, 5, 14);
        graph.kruskal();
        graph.prim();
        graph.bfs(0);
        graph.dfs(0);
        graph.topSort();
        graph.bf(0);
        graph.dijkstra(0);
    }
}
