/* Module SM_1118_Bot_Traversal

This module is responsible for the Bot Traversal on the arena.
Path Planning utilize the Dijkstra Algorithm and uses the information
of the arena which is stored as two-dimensional graph in the Planner to
move the bot effectively.

This Module also takes the line following algorithm. So, that the bot can
executes synchronously with the Path Planner.

Bot Traversal Design
Input  : clk   - 3.125Mhz Clock
         color - 2-bit color Value to pick package
         left_value, center_value, right_value - line follower sensor Outputs
Output : taskend  - inidicates task Completion
         statusno - Status No of the Nodes being traversed
         farmland - Indicates the present farmland
         msg_type - Indicates the message type to be transmitted
         enbem    - enable electromagnet
         movement - Motor Control
*/
// Module Declaration
module SM_1118_Bot_Traversal(
    input [1:0] color,
    input  clk,
    input  [11:0] left_value, center_value, right_value,
    output reg taskend,
    output reg [1:0] statusno, farmland, msg_type,
    output reg enbem,
    output reg [3:0] movement
);

// Declaring Constants
parameter nodes = 28, inf = 99;

// Declaring State Machine Parameters
parameter init = 1'b0, plan = 1'b1;

// 2 Dimensional Graph which will hold the information of SM Arena
// Neccessary Variables required for Djikstra Algorithm are Declared
// Also All Necessary Variable declarations are made Here
reg [21:0]  delay_counter_stop = 0, node_delay_counter = 0,
            turn_delay_counter = 0, push_delay_counter = 0;
reg [7:0] cost_graph[0:27][0:27], distance[0:27], mindistance;
reg [4:0] graph[0:27][0:27], corner[0:7], predictions[0:27];
reg [4:0] startnode, targetnode, pathnode, nextnode, lastnode;
reg [4:0] warehouse[0:6], targets[0:7], i, j, k, l, m, swap, count;
reg [4:0] path[0:27], flag1, flag2, dir, bot_face, node;
reg [3:0] turn[0:27], rotate = 0;
reg [2:0] target = 0;
reg [1:0] direction[0:27][0:27], status[0:27], packages[0:27], field[0:27];
reg [1:0] statusnode[0:27][0:27], farmnode[0:27][0:27], nodesvisited[0:27];
reg [1:0] supply[0:6], temp_color;
reg node_flag = 0, before_turn_flag = 0, turn_flag = 0, after_turn_flag = 0;
reg planner = 0, plan_done = 0, push_flag = 0, present=init, complete, next;

// Get required Turn needed for obtaining Goal
function [2:0] get_turn(input [2:0] bot_face, dir); begin
    get_turn = 0;
    if(bot_face == dir) get_turn = 0;
    else if(bot_face != dir) begin
        if(bot_face == 0) begin
            if(dir == 1) get_turn = 5;
            else if(dir == 2) get_turn = 6;
            else if(dir == 3) get_turn = 7;
        end
        else if(bot_face == 1) begin
            if(dir == 0) get_turn = 6;
            else if(dir == 2) get_turn = 7;
            else if(dir == 3) get_turn = 5;
        end
        else if(bot_face == 2) begin
            if(dir == 0) get_turn = 5;
            else if(dir == 1) get_turn = 7;
            else if(dir == 3) get_turn = 6;
        end
        else if(bot_face == 3) begin
            if(dir == 0) get_turn = 7;
            else if(dir == 1) get_turn = 6;
            else if(dir == 2) get_turn = 5;
        end
    end
end
endfunction

initial begin
    statusno = 0; msg_type = 1; next = 0; enbem = 0;
    complete = 0; taskend = 0; bot_face = 3;
end

/* Movements Used             Delays Used
movement   bot_response       time(s)     Counter Value
    0         Stop            0.25        781250
    1         Forward         0.4         1250000
    2         Right           0.50        1562500
    3         Left            0.75        2343750
    4         Reverse         1.00        3125000
    5         turn Right
    6         turn Left                                   */

// Path Planner
always @(posedge planner) begin
    // Variable Intialization
    pathnode = 0; count = 1; flag1 = 0; flag2 = 0;
    complete = 0; lastnode = 0; target = 0; m = 0;
    temp_color = 3; // For this Task
    // Declaring All the Parameters to be Zero at Start
    for(i=0; i<nodes; i=i+1'b1) begin
        status[i] = 0; path[i] = 0; field[i] = 0; turn[i] = 0;
    end
    for(i=0;i<nodes;i=i+1'b1) begin
        for(j=0;j<nodes;j=j+1'b1) begin
            graph[i][j] = 0; direction[i][j] = 0; statusnode[i][j] = 0; farmnode[i][j] = 0;
        end
    end

    // For status indication
    statusnode[13][12] = 2; statusnode[12][11] = 3; statusnode[15][14] = 1;
    statusnode[8][9] = 2; statusnode[6][7] = 1; statusnode[3][15] = 2;
    statusnode[5][4] = 1; statusnode[5][0] = 1; statusnode[1][2] = 2;

    farmnode[13][12] = 0; farmnode[12][11] = 0; farmnode[15][14] = 0;
    farmnode[8][9] = 2; farmnode[6][7] = 2; farmnode[3][15] = 3;
    farmnode[5][4] = 3; farmnode[5][0] = 1; farmnode[1][2] = 1;

    // Declaring Notched/Corner Nodes, so that the bot will 
    // not make change in direction
    corner[0] = 0; corner[1] = 1; corner[2] = 5; corner[3] = 6;
    corner[4] = 10; corner[5] = 11; corner[6] = 12; corner[7] = 16;

    // Decalaring Warehouse Supply Pick Node Numbers
    warehouse[0] = 26; warehouse[1] = 25; warehouse[2] = 27; warehouse[3] = 24;
    warehouse[4] = 28; warehouse[5] = 23; warehouse[6] = 22;

    // Graph Created for SM Arena
    // Weights for All Nodes are assigned below.
    graph[0][1] = 4; graph[0][5] = 3; graph[1][0] = 4; graph[1][2] = 2; graph[1][16] = 8;
    graph[2][1] = 2; graph[2][3] = 1; graph[3][2] = 1; graph[3][4] = 2; graph[3][20] = 2;
    graph[4][3] = 2; graph[4][5] = 3; graph[5][4] = 3; graph[5][0] = 3; graph[3][15] = 4;
    graph[5][6] = 5; graph[6][5] = 5; graph[6][7] = 2; graph[7][6] = 2; graph[6][10] = 7;
    graph[7][8] = 3; graph[8][7] = 3; graph[8][9] = 2; graph[9][8] = 2; graph[8][15] = 2;
    graph[10][9]  = 1; graph[9][10]  = 1; graph[15][8]  = 2; graph[16][1]  = 8; graph[20][3]  = 2;
    graph[10][11] = 2; graph[11][12] = 6; graph[16][17] = 1; graph[16][13] = 4; graph[21][20] = 1;
    graph[11][10] = 2; graph[12][11] = 6; graph[12][13] = 2; graph[13][12] = 2; graph[13][14] = 4;
    graph[13][16] = 4; graph[14][13] = 4; graph[14][15] = 3; graph[15][14] = 3; graph[20][19] = 1;
    graph[17][16] = 1; graph[17][24] = 1; graph[17][25] = 1; graph[17][18] = 1; graph[20][21] = 1;
    graph[18][17] = 1; graph[18][23] = 1; graph[18][26] = 1; graph[18][19] = 1; graph[24][17] = 1;
    graph[19][18] = 1; graph[19][22] = 1; graph[19][27] = 1; graph[19][20] = 1; graph[23][18] = 1;
    graph[22][19] = 1; graph[25][17] = 1; graph[26][18] = 1; graph[27][19] = 1; graph[15][3]  = 4;
    graph[10][6] = 7;

    // Direction Graph which will contain the informations of direction between 
    // multiple subsequent Nodes
    // North - 0 East - 1 West - 2 South - 3
    direction[0][1] = 1; direction[0][5] = 3; direction[1][0] = 2; direction[1][2] = 3; direction[1][16] = 1;
    direction[2][1] = 0; direction[2][3] = 3; direction[3][2] = 0; direction[3][4] = 2; direction[3][20] = 1;
    direction[4][3] = 1; direction[4][5] = 2; direction[5][4] = 1; direction[5][0] = 0; direction[3][15] = 3;
    direction[5][6] = 3; direction[6][5] = 0; direction[6][7] = 1; direction[7][6] = 2; direction[6][10] = 3;
    direction[7][8] = 1; direction[8][7] = 2; direction[8][9] = 3; direction[9][8] = 0; direction[8][15] = 0;
    direction[10][9]  = 0; direction[9][10]  = 3; direction[15][8]  = 3; direction[16][1]  = 0; direction[20][3]  = 2;
    direction[10][11] = 1; direction[11][12] = 1; direction[16][17] = 2; direction[16][13] = 3; direction[21][20] = 0;
    direction[11][10] = 2; direction[12][11] = 3; direction[12][13] = 0; direction[13][12] = 3; direction[13][14] = 2;
    direction[13][16] = 0; direction[14][13] = 1; direction[14][15] = 2; direction[15][14] = 1; direction[20][19] = 1;
    direction[17][16] = 1; direction[17][24] = 3; direction[17][25] = 0; direction[17][18] = 2; direction[20][21] = 3;
    direction[18][17] = 1; direction[18][23] = 3; direction[18][26] = 0; direction[18][19] = 2; direction[24][17] = 0;
    direction[19][18] = 1; direction[19][22] = 3; direction[19][27] = 0; direction[19][20] = 2; direction[23][18] = 0;
    direction[22][19] = 0; direction[25][17] = 3; direction[26][18] = 3; direction[27][19] = 3; direction[15][3]  = 0;
    direction[10][6] = 2;

    // Creating Target Array
    targets[0] = 17; targets[1] = 7; targets[2] = 8; targets[3] = 4;
    targets[4] = 23; targets[5] = 8;

    // Creating Supply Packages Position
    // Only for this Task
    supply[0] = 1; supply[1] = 3; supply[2] = 2; supply[3] = 0;
    supply[4] = 1; supply[5] = 3; supply[6] = 2;

	 for(i=0; i<7; i=i+1'b1) begin
        if(temp_color == supply[i]) targets[4] = warehouse[i];
    end
	 
    while(target < 5) begin
        startnode = targets[target]; targetnode = targets[target+1];
        // check whether the target is within warehouse
        // Initializing Variables
        target = target + 1'b1;
        lastnode = 0; complete = 0;
        nextnode = 0; mindistance = 0;
        // Swaping Variables
        swap = startnode - 1'b1;
        startnode = targetnode - 1'b1;
        targetnode = swap;
        // For Non-Connected nodes make the distances to be infinite
        // else update the value for cost graph.
        for(i=0;i<nodes;i=i+1'b1) begin
            for(j=0;j<nodes;j=j+1'b1) begin
                cost_graph[i][j] = 0;
                if(graph[i][j]==0) cost_graph[i][j]=inf;
                else cost_graph[i][j]=graph[i][j];
            end
            predictions[i] = 0; distance[i] = 0;
        end
        // Exploring Graph
        for(i=0;i<nodes;i=i+1'b1) begin
            distance[i]=cost_graph[startnode][i];
            predictions[i]=startnode; nodesvisited[i]=0;
        end
        // Computing Shortest Path for each Node from startNode
        distance[startnode]=0; nodesvisited[startnode]=1; count=1;
        while(count<nodes-1) begin
            mindistance=inf;
            for(i=0; i<nodes; i=i+1'b1) begin
                if(distance[i]<mindistance&&!nodesvisited[i]) begin
                    mindistance=distance[i]; nextnode=i;
                end
            end
            nodesvisited[nextnode]=1;
            for(i=0; i<nodes; i=i+1'b1) begin
                if(!nodesvisited[i]) begin
                    if(mindistance+cost_graph[nextnode][i]<distance[i]) begin
                        distance[i]=mindistance+cost_graph[nextnode][i];
                        predictions[i]=nextnode;
                    end
                end
            end
            count = count + 1'b1;
        end
        if(targetnode!=startnode) begin
            // Get Result
            // The result will contain the shortest distance and
            // all the subset of nodes to be traversed.
            i = 0; j = 0; k = 0; l = 0;
            $display("\nDistance of Node %0d to Node %0d is %0d", targetnode+1, startnode+1, distance[targetnode]);
            $display("Node <- %0d ", targetnode+1);
            j = targetnode;
            while(complete!=1) begin
                l = predictions[j];
                if(lastnode == 0) begin
                    // Here Direction of Turn will be obtained which is based on the
                    // direction of the bot traversing node and its subscequent node
                    dir = direction[j][l];
                    turn[m] = get_turn(bot_face, dir);
                    // Corner Conditions are Added Here
                    for(k=0; k<8; k=k+1'b1) begin
                        if(j == corner[k]) flag1 = 1;
                        if(l == corner[k]) flag2 = 1;
                    end
                    // Updating Direction of the Bot
                    if(flag1 == 1 && flag2 == 1) begin
                        k = predictions[l]; bot_face = direction[l][k];
                        if(k == l) bot_face = direction[j][l];
                    end
                    else bot_face = dir;
                    flag1 = 0; flag2 = 0;
                    // Status Indication Number
                    if(statusnode[j][l]!=0) begin
                        status[m] = statusnode[j][l]; field[m] = farmnode[j][l]; 
                    end
                    // Get the Count of Nodes for Bot Traversal
                    pathnode = pathnode + 1'b1; m = m + 1'b1; j = predictions[j];
                    $display("<- %0d ", j+1);
                end
                else if(lastnode == 1) begin
                    turn[m] = 8; lastnode = 0; complete = 1;
                end
                if(j==startnode) lastnode = 1;
                path[m] = pathnode;
                if(turn[m] == 8 && startnode == 7) begin
                    path[m+1] = pathnode + 1'b1; turn[m+1] = 8;
                    if(status[m] == 0) turn[m] = 6;
                    else turn[m] = 5;
                end
            end
        end
    end
    $display("\nBot should Turn at these Nodes 5-Right 6-Left 7-Uturn 4-Stop\n");
    $display("FarmLand 0-MaizeTerrain 1-PaddyPlains 2-NuttyGround 3-VegetableGarden\n");
    $display("\nNodeNo(BLF) Turn StatusNo FarmLand");
    for(i=0; i<nodes; i=i+1'b1) begin
        if(path[i]!=0) begin
            $display("      %0d      %0d       %0d       %0d", path[i], turn[i], status[i], field[i]);
        end
    end
    plan_done = 1;
end

always@(posedge clk) begin
    // Make the Bot to Follow the planned Path
    case(present)
        init:
            begin
                planner = 0; enbem = 0; movement = 0;
                if(next == 0) present = plan;
                else if(next == 1) taskend = 1;
            end
        plan:
            begin
                planner = 1; next = 1; 
                if(rotate == 8) begin
                    present = init; msg_type = 3; enbem = 0;
                end
				else begin
                    farmland = field[node]; statusno = status[node];
    				if(turn[node+1] == 7) enbem = 1;
					else if (turn[node] == 7) msg_type = 2;
				end
            end
    endcase
    // Start Line Following after the planner plans a path.
    if(plan_done == 1 && taskend == 0) begin
        // If the Line follower sensors white light intensity
        // then first push the bot forward for 1s else stop the bot
        if (turn_flag == 0 && after_turn_flag == 0) begin
            if (left_value < 300 && center_value < 300 && right_value < 300) begin
                if (delay_counter_stop < 3125000) begin
					movement = 1; delay_counter_stop = delay_counter_stop + 1'b1;
                end
            end
            // If the Line follower sensor senses white, black and white light intensity
            // then follow the black line
            else if (left_value < 350 && center_value > 250 && right_value < 350) begin
                movement = 1; delay_counter_stop = 0;
            end
            // If the Line Follower sensor senses black light intensity on the right sensor
            // then rotate the bot right
            else if (left_value < 300 && right_value > 150) begin
				movement = 2; delay_counter_stop = 0;
            end
            // If the Line Follower sensor senses black light intensity on the left sensor
            // then rotate the bot left
            else if (left_value > 150 && right_value < 300) begin
				movement = 3; delay_counter_stop = 0;
            end
			else delay_counter_stop = 0;
        end
        // Check whether the bot is in the node or not
        // The bot will take the required turn if it senses the input node
        if (left_value > 150 && center_value > 300 && right_value > 150 && turn_flag == 0) begin
            if (node_flag == 0 && turn_flag == 0 && after_turn_flag == 0) begin
				node_flag = 1; node = node + 1'b1; rotate = turn[node]; 
                if (rotate != 7) push_flag = 1;
                if(rotate == 5 || rotate == 6 || rotate == 7) before_turn_flag = 1;
            end
        end
        // A 0.75 second delay added to make sure not to add
        // the same node multiple times
        if (node_flag == 1) begin
            node_delay_counter = node_delay_counter + 1'b1;
			if (node_delay_counter > 2343750) begin
				node_delay_counter = 0;
				node_flag = 0;
			end
		end
        // A small Delay to make bot move forward for 0.25s
        // after node detection
        if (push_flag == 1) begin
            movement = 1; push_delay_counter = push_delay_counter + 1'b1;
            if (push_delay_counter > 781250) begin
                push_flag = 0;
                push_delay_counter = 0;
            end
        end
        // Align the bot before taking the turn
        if (before_turn_flag == 1) begin
            movement = 1; turn_delay_counter = turn_delay_counter + 1'b1;
            if (turn_delay_counter > 500250) begin
                turn_delay_counter = 0; before_turn_flag = 0;
                turn_flag = 1;
            end
        end
        // Turn the bot
        else if (turn_flag == 1) begin
            movement = rotate; turn_delay_counter = turn_delay_counter + 1'b1;
            if (turn_delay_counter > 781250) begin
                turn_delay_counter = 0; turn_flag = 0;
                after_turn_flag = 1;
            end
        end
        // Turn the bot until the bot detects the line
        else if (after_turn_flag == 1) begin
            movement = rotate;
			if (rotate == 5 && right_value > 300) after_turn_flag = 0;
			else if ((rotate == 6 || rotate == 7) && left_value > 300) after_turn_flag = 0;
				  
        end
    end
end

endmodule
