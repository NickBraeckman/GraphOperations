import java.util.LinkedList;
import java.util.Queue;

public class MaxFlow {

    public static int V;

    public boolean bfs(int[][] graph, int s, int t, int parent[]){
        boolean visited[] = new boolean[V];

        Queue<Integer> Q = new LinkedList<>();
        Q.add(s);
        visited[s] = true;
        parent[s] = -1;
        while (!Q.isEmpty() ){
            int u = Q.poll();
            for (int v=0; v < V; v++){
                if (!visited[v] && graph[u][v] > 0){
                    Q.add(v);
                    parent[v] = u;
                    visited[v] = true;
                }
            }
        }
        return (visited[t]);
    }

    public int fordFulkerson(int[][] graph, int s, int t){
        int u,v;
        int parent[] = new int[V];
        int maxFlow = 0;

        // init the residual network
        int Gf[][] = new int[V][V];
        for (u=0; u < V; u++){
            for (v=0; v < V; v++){
                Gf[u][v] = graph[u][v];
            }
        }

        while (bfs(Gf, s, t , parent)){

            // find the minimum residual capacity cf of the edges
            // along the path filled by bfs
            int f = Integer.MAX_VALUE;

            // every iteration v becomes his parent and so on
            for (v= t; v!=s; v=parent[v]){
                u = parent[v];
                f = Math.min(f, Gf[u][v]);
            }

            // update residual capacities of the edges
            // reverse edges along the path
            for (v=t; v != s; v = parent[v]){
                u = parent[v];
                Gf[u][v] -= f;
                Gf[v][u] += f;
            }

            maxFlow += f;
        }

        return maxFlow;
    }

    public static void main(String[] args) {
        V = 6;
        int [][] graph = new int[][]{
                {0, 16, 13, 0, 0, 0},
                {0, 0, 10, 12, 0, 0},
                {0, 4, 0, 0, 14, 0},
                {0, 0, 9, 0, 0, 20},
                {0, 0, 0, 7, 0, 4},
                {0, 0, 0, 0, 0, 0}
        };

        MaxFlow maxFlow = new MaxFlow();
        int m = maxFlow.fordFulkerson(graph,0,5);
        System.out.println("The max flow is: " + m);
    }
}
