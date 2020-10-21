# aqmaps
ILP - CW2 Drone flight

# Algorithm Pseudocode
1. Create a matrix of vertices V that follow a pattern of equilateral triangles with side length = 0.0003
2. Add edges E to each trio of vertices in V that forms a triangle.
3. For each sensor s, add the vertex v from V that is closest (in euclidean distance) to s, to 'must-visit points' M. Also add starting point K to M.
4. For each m in M, use Dikjstra's algorithm to find shortest path to all other must-visit points. Create an proxy edge p for this path with weight equal to the number of edges used from E. Add p to list that represents all 'proxy edges' P.
5. Create a graph H with vertices M and edges P.
6. Run Hamilton algorithm on H starting at point K (from 2).
7. Back link shortest path output from 6 to edges E. 

