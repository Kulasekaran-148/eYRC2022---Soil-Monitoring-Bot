/* Module SM1118_Algorithm

This module will plan a custom path from nodeA to nodeB using djikstra Algoritm
and identifies the nodes on which bot needs to stop/turn.
Also, this module outputs the status identifier which is associated with the 
particular node of the farmland.
Algorithm Design
Input : clk  - 3.125Mhz input clock
        node - Current Node of the Bot
Output: turn      - Direction of the turn
        statusno  - SI Number for Msg Transmission
        turn_flag - Flag to take turn or not */

// Module Declaration
 module SM1118_Algorithm(
    input clk,
    input [5:0] node,
    output reg [1:0] statusno, msg_type, field,
    output reg [5:0] nodex,
    output reg [2:0] turn
 );
// 2 Dimensional Graph which will hold the information of Maize Terrain
// Neccessary Variables required for Djikstra Algorithm are Declared
reg [10:0] graph[0:7][0:7], cost_graph[0:7][0:7];
reg [10:0] distance[0:7], predictions[0:7], direction[0:7], nodesvisited[0:7], mindistance;
reg [5:0] path[0:7], turns[0:7], status[0:7];
// Declaring Variables
reg [3:0] startnode, targetnode;
reg [3:0] swap, i, j, k, t, count, nextnode;
// Declaring Constants
parameter max = 8, inf=99;
// Graph Declaration only works on SystemVerilog
// integer graph [7:0][7:0] = {{0, 2, 0, 0, 0, 0, 0, inf1}, {2, 0, 1, 0, 0, 0, 0, 0}, 
//                             {0, 1, 0, 2, 0, 0, 0, 0}, {0, 0, 2, 0, 1, 0, 0, 0}, 
//                             {0, 0, 0, 1, 0, 3, 0, 0}, {0, 0, 0, 0, 3, 0, 4, 0}, 
//                             {0, 0, 0, 0, 0, 4, 0, 2}, {6, 0, 0, 0, 0, 0, 2, 0}};

initial begin

    // Declaring Start and Target Nodes
    startnode = 1; targetnode = 8; t = 0;

    // Declaring Default Directions for the Nodes of Maize Terrain
    // Value        Direction
    //   0            North
    //   1            East
    //   2            West
    //   3            South
    direction[0] = 2; direction[1] = 2; direction[2] = 0; direction[3] = 0;
    direction[4] = 0; direction[5] = 1; direction[6] = 1; direction[7] = 3;

    // For this task msg_type is made to be 1
    // As it only detects color
    msg_type = 1;
    // As the bot travels only on Maize Terrain
    // field is set to 0
    field = 0;

    // Declaring Status Number of the particular Nodes
    // By Default all the Nodes is initialized with zero
    for(i=0;i<max;i=i+1) status[i] = 0;
    // Only the Nodes containing Status are intialized with 
    // appropriate SI Numbers
    status[0] = 3; status[5] = 1; status[7] = 2; 

    // Graph Creation for Maize Terrain
    // Note: For any loops having comparison variables doesn't work on
    // quartus compilers so the condition i<nodes is replaced by 8
    for(i=0;i<max;i=i+1) begin
        for(j=0;j<max;j=j+1) begin
            graph[i][j] = 0;
        end
    end

    // Made 1 to 8 as infinity so that the bot doesn't track reverse path
    // Weights for All Nodes are assigned below.
    graph[1][0] = 2; graph[7][0] = inf; graph[0][1] = 2; graph[2][1] = 1;
    graph[1][2] = 1; graph[3][2] = 2; graph[2][3] = 2; graph[4][3] = 1;
    graph[3][4] = 1; graph[5][4] = 3; graph[4][5] = 3; graph[6][5] = 4;
    graph[5][6] = 4; graph[7][6] = 2; graph[0][7] = 6; graph[6][7] = 2;

    // Swaping Variables
    swap = startnode-1;
    startnode = targetnode-1;
    targetnode = swap;

    // For Non-Connected nodes make the distances to be infinite
    // else update the value for cost graph.
    for(i=0;i<max;i=i+1) begin
        for(j=0;j<max;j=j+1) begin
            if(graph[i][j]==0) cost_graph[i][j]=inf;
            else cost_graph[i][j]=graph[i][j];
        end
    end

    // Exploring Graph
    for(i=0;i<max;i=i+1) begin
        distance[i]=cost_graph[startnode][i];
        predictions[i]=startnode;
        nodesvisited[i]=0;
    end
    distance[startnode]=0;
    nodesvisited[startnode]=1;
    count=1;

    // Computing Shortest Path for each Node from startNode
    while(count<max) begin
        mindistance=inf;
        for(i=0; i<max; i=i+1) begin
            if(distance[i]<mindistance&&!nodesvisited[i]) begin
                mindistance=distance[i];
                nextnode=i;
            end
        end
        nodesvisited[nextnode]=1;
        for(i=0; i<max; i=i+1) begin
            if(!nodesvisited[i]) begin
                if(mindistance+cost_graph[nextnode][i]<distance[i]) begin
                        distance[i]=mindistance+cost_graph[nextnode][i];
                        predictions[i]=nextnode;
                end
            end
        end
        count = count + 1;
    end

    // Get Result
    // The result will contain the shortest distance and
    // all the subset of nodes to be traversed
    if(targetnode!=startnode) begin
        k = 0;
        $display("Distance of node%0d = %0d", targetnode+1, distance[targetnode]);
        $display("Node <- %0d Direction  <- %0d", targetnode+1, direction[targetnode]);
        j = targetnode;
        // Condition should be j!=startnode but as there is a compiler issue
        // condition replaced as j!=7 where 7 is targetnode-1
        while(j!=max-1) begin
            j = predictions[j];
            $display("Node <- %0d Direction  <- %0d", j+1, direction[j]);
            // Here Direction of Turn will be obtained which is based on the 
            // direction of the bot traversing node and its subscequent node
            // 5 - Right Turn, 6 - Left Turn
			if((j+1)!=max) begin
				if(direction[j]!=direction[j+1]) begin
					path[k] = j+1;
					if(direction[j] == 0) begin
					    if(direction[j+1] == 1) turns[k] = 5;
						if(direction[j+1] == 2) turns[k] = 6;
					end
					else if(direction[j] == 1) begin
						if(direction[j+1] == 0) turns[k] = 6;
						if(direction[j+1] == 3) turns[k] = 5;
					end
					else if(direction[j] == 2) begin
						if(direction[j+1] == 0) turns[k] = 5;
						if(direction[j+1] == 3) turns[k] = 6;
					end
					else if(direction[j] == 3) begin
						if(direction[j+1] == 1) turns[k] = 6;
						if(direction[j+1] == 2) turns[k] = 5;
					end
					k = k + 1;
				end
		    end
        end
    end
    $display("Final Set of Nodes and direction of turn are");
    for(i=0; i<max; i=i+1) begin
        $display("%0d %0d", path[i], turns[i]);
    end
end

always @(posedge clk) begin
    if(node <= 2) begin
		nodex = path[t]; turn = turns[t];
    end
	 else if(node <= 5) begin
		nodex = path[t+1]; turn = turns[t];
	 end
	 else if(node <= 7) begin
		nodex = path[t+2]; turn = turns[t];
    end
    if(node == 0) statusno = status[0];
    else if(node == 5) statusno = status[5];
    else if(node == 7) statusno = status[7];
end

endmodule