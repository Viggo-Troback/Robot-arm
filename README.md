# SEGMENTED ROBOT ARM PATH FINDER
Allows you to design a simple arm and load it as its segments in .DXF format.
From there you can calculate all reachable points given the freedom of moments by building a virtual arm. Follow the config structure and store these points as both a KD-tree, graph with neighbours (x,y):[(x,y),(x,y)] and points dict (x,y):[combo1, combo2] as there mighht be more than one way to reach a point. 
Using this and an input text it can calculate the closest approximate points and paths between them and visualize it in a graph and how the arm reaches a point within the graph.

## Future work

Adding visualisation of how the arm draws the path.

Adding two seperate paths- path to draw, path bewtween draw points. At the moment only draw path is implemented. 

Improving the text_to_points function. At the moment it only plots the points at the edges of long straight lines making letters like "L" and "V" very hard to approximate since the distance between vertices is very long.

Way to many stacked loops in the finding closest neighbours function. Too slow even for small sets of points. 

## Known bugs