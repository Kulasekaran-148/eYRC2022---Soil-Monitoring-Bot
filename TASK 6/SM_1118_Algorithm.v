/*
Team Id    : 1118
Author List: Hari Vikinesh M, Kulasekaran S, Jerish Abijith Singh  A S,
             Aryan Vinunath
File Name  : SM_1118_Algorithm.v
Theme      : Soil Monitoring Bot

Module SM_1118_Algorithm

This module is the implementation of path planning using the Dijkstra 
Algorithm. The whole information about the arena is provided to the 
algorithm in the form of single or two dimensional arrays. 

Bot Traversal Design
Input  : clk       - 3.125Mhz ADC Clock
         clk_50m   - 50Mhz Clock
         rx_index  - Index Number to append the rx_color to the supply package array
         rx_color  - 2-bit color value of supply color
         color     - 2-bit color Value to pick package
         tx_done   - Status of Xbee Transmission
         node      - 5 bit value denoting no. of nodes traversed

Output : taskend   - Inidicates task Completion
         offind    - Off index to switch off led
         blockflag - Disable color Detection if the bot is on pick or place operation
         statusno  - Status No of the Nodes being traversed
         tx_color  - Transmit Color
         tx_start  - Initialize Xbee Transmission
         farmland  - Indicates the present farmland
         msg_type  - Indicates the message type to be transmitted
         enableem  - enable electromagnet
         rotate    - Motor Control 
         rxdone    - Supply Receiving Status    */

// Module Declaration
module SM_1118_Algorithm(
    input  tx_done,
    input  [1:0] color,
    input  clk,
    input  [4:0] node,
    input  [3:0] rx_index,
    input  [1:0] rx_color,
    input  clk_50m,
    output reg taskend,
    output reg [1:0] offind,
    output reg blockflag,
    output reg [1:0] statusno, tx_color,
    output reg tx_start,
    output reg [1:0] farmland, msgtype,
    output reg [2:0] rotate,
    output reg rxdone,
    output reg [1:0] enableem
);

// Declaring Constants
parameter nodes = 29, max = 15, init = 3'b000, s0 = 2'b00, s1 = 2'b01, width = 27,
          choosetarget = 3'b001, validate = 3'b010, s2 = 2'b10, predict = 1'b0,
          dijkstra = 3'b011, planner = 3'b100, tracker = 3'b101, compute = 1'b1;

// Declaring 2 Dimensional arrays and variables holding the arena information
// which is provided as input to the Dijikstra's algorithm. 
reg [7:0]  costgraph[0:28][0:28], distance[0:28], mindist, inf = 99;
reg [4:0]  prediction[0:27], nodex[0:27], corner[0:7], count, k = 0;
reg [4:0]  warehouse[0:7], startnode, targetnode, nextnode, temp_index = 0;
reg [4:0]  targets[0:14], i = 0, j = 0, l = 0, swap, ptarget, itarget[0:2];
reg [3:0]  traverseindex = 0;
reg [2:0]  turn[0:27], index = 0, track = 0, present = init, sic = 0;
reg [1:0]  statusnode[0:28][0:28], farmnode[0:28][0:28], field[0:27];
reg [1:0]  istatus[0:2], ifarm[0:2], icolor[0:2],   deposition[0:27];
reg [1:0]  directions[0:28][0:28], nodesvisited[0:27], status[0:27],  botface;
reg [1:0]  packages[0:7], direct, algorithm = s0, tempdir = 0, placetrack = 0;
reg [1:0]  tempstatus = 0, tempfarm = 0, tempcolor = 0, picktrack = 0, ci = 0;

// Neccessary Flags are Declared
reg rxinitflag = 1, complete = 0, flag1 = 0, flag2 = 0, next = 0, pick = 0;
reg initdone = 1, tempflag = 0, plannerx = predict, siidmax = 0, place = 0, diru = 0;
reg once = 0, done = 0, last = 0, reset = 0, clear = 0, txm1 = 0, txm2 = 0, txm3 = 0;

always @(posedge clk_50m) begin
    // Initializing packages Array to zero,
    // before Xbee starts receiving SPM
    if(rxinitflag == 1) begin
        if(index < 7) begin
            packages[index] = 0; index = index + 1'b1;
            if(index < 3) begin
                istatus[index] = 0; ifarm[index] = 0; icolor[index] = 0; itarget[index] = 0;
            end
        end
        else begin
            rxinitflag = 0; index = 0; rxdone = 0;
            statusno = 0; msgtype = 0; farmland = 0;
        end
    end
    else if(rxdone == 0) begin
        // If Xbee receives the message, then map
        // them appropriately to the packages array.
        if(rx_index < 8) begin
            packages[rx_index] = rx_color;
            if(rx_index == 7) index = 7;
        end
        // If all the input colors are received, then receiving is completed.
        if(index == 7) rxdone = 1;
    end
    // Update the color into appropriate array to make the bot traverse the field.
    // untill it detects atleast three color.
    if(pick == 0 && place == 0) begin
        farmland = field[node]; statusno = status[node];
    end
    if(pick == 1 || place == 1) blockflag = 1;
    else blockflag = 0;
    if(color != 0 && statusno != 0 && pick == 0 && place == 0) begin
        if(ci < 3 && (tempstatus != statusno || tempfarm != farmland || tempcolor != color)) begin
            istatus[ci] = statusno; ifarm[ci] = farmland; itarget[ci] = startnode + 1'b1;
            icolor[ci] = color; tempstatus = statusno; tempfarm = farmland; tempcolor = color;
            ci = ci + 1'b1; sic = sic + 1'b1; txm1 = 1; tx_color = color;
        end
        // If three colors are identified then make the siidentification reached max
        else if(sic == 3 || sic == 6) begin
            siidmax = 1; ci = 0;
        end
    end
    // If reset is enabled remove all the values in the identification array
    if(reset == 1) begin
        if(ci < 3) begin
            istatus[ci] = 0; ifarm[ci] = 0; icolor[ci] = 0;
            ci = ci + 1'b1; siidmax = 0; done = 0; itarget[ci] = 0;
        end
        else begin
            ci = 0; done = 0;
        end
    end
    // Clears the Picked Supply Package
    else if(clear == 1) packages[temp_index] = 0;
    // Chooses the message Format for Xbee Transmission
    if(txm1 == 1) begin
        tx_start = 1; msgtype = 1; txm1 = 0;
    end
    if(txm2 == 1 && once == 0) begin
        tx_start = 1; tx_color = icolor[picktrack-1]; msgtype = 2; once = 1;
        statusno = istatus[picktrack-1]; farmland = ifarm[picktrack-1];
    end
    else if(txm3 == 1 && once == 1) begin
        tx_start = 1; tx_color = icolor[placetrack]; msgtype = 3; once = 0;
        statusno = istatus[placetrack]; farmland = ifarm[placetrack];
    end
    if(tx_done == 1) tx_start = 0;
end

always @(posedge clk) begin
    // Initializing All parameters to be zero/Constants at the start
    if(initdone == 1) begin
        if(i<nodes) begin
            deposition[i] = 0;
            if(j<nodes) begin
                directions[i][j] = 0; statusnode[i][j] = 0;
                farmnode[i][j] = 0; costgraph[i][j] = inf;
                j = j + 1'b1;
            end
            else begin
                j = 0; i = i + 1'b1;
            end
        end
        else begin
            enableem = 0; i = 0; j = 0; l = 0;
            taskend = 0; rotate = 0; next = 0;
            botface = 2; initdone = 0;  k = 0;
            ptarget = 0; tempflag = 0; offind = 3;

            // For status identification
            statusnode[13][12] = 2; statusnode[12][11] = 3; statusnode[15][14] = 1;
            statusnode[8][9] = 2; statusnode[6][7] = 1; statusnode[3][15] = 2;
            statusnode[5][4] = 1; statusnode[5][0] = 1; statusnode[1][2] = 2;

            farmnode[13][12] = 0; farmnode[12][11] = 0; farmnode[15][14] = 0;
            farmnode[8][9] = 2; farmnode[6][7] = 2; farmnode[3][15] = 3;
            farmnode[5][4] = 3; farmnode[5][0] = 1; farmnode[1][2] = 1;

            // Declaring Notched/Corner Nodes, to maintain the correct 
            // bot traversing direction when traversing in the corners
            corner[0] = 0; corner[2] = 5; corner[4] = 10; corner[6] = 12;
            corner[1] = 1; corner[3] = 6; corner[5] = 11; corner[7] = 16;

            // Declaring Warehouse Supply Pick Node Numbers
            warehouse[0] = 31; warehouse[1] = 26; warehouse[2] = 25; warehouse[3] = 27;
            warehouse[4] = 24; warehouse[5] = 28; warehouse[6] = 23; warehouse[7] = 22;

            // Declaring Deposition Zone direction update Values
            deposition[0] = 0; deposition[2] = 1; deposition[4] = 0; deposition[15] = 1; deposition[14] = 0;
            deposition[7] = 0; deposition[9] = 1; deposition[12] = 1; deposition[11] = 3;

            // Declaring traverseindex Array
            targets[0] = 17; targets[1] = 13; targets[2] = 12; targets[3] = 7; targets[4] = 8;
            targets[5] = 7; targets[6] = 6; targets[7] = 5; targets[8] = 6; targets[9] = 1;
            targets[10] = 3; targets[11] = 16; targets[12] = 15; targets[13] = 16; targets[14] = 10;

            // Graph Created for SM Arena
            // Weights for All Nodes are assigned below.
            costgraph[0][1] = 4; costgraph[0][5] = 3; costgraph[1][0] = 4; costgraph[1][2] = 2; costgraph[1][16] = 8;
            costgraph[2][1] = 2; costgraph[2][3] = 1; costgraph[3][2] = 1; costgraph[3][4] = 2; costgraph[3][20] = 2;
            costgraph[4][3] = 2; costgraph[4][5] = 3; costgraph[5][4] = 3; costgraph[5][0] = 3; costgraph[3][15] = 4;
            costgraph[5][6] = 5; costgraph[6][5] = 5; costgraph[6][7] = 2; costgraph[7][6] = 2; costgraph[6][10] = 7;
            costgraph[7][8] = 3; costgraph[8][7] = 3; costgraph[8][9] = 2; costgraph[9][8] = 2; costgraph[8][15] = 2;
            costgraph[10][9]  = 1; costgraph[9][10]  = 1; costgraph[15][8]  = 2; costgraph[16][1]  = 8; costgraph[20][3]  = 2;
            costgraph[10][11] = 2; costgraph[11][12] = 6; costgraph[16][17] = 1; costgraph[16][13] = 4; costgraph[21][20] = 1;
            costgraph[11][10] = 2; costgraph[12][11] = 6; costgraph[12][13] = 2; costgraph[13][12] = 2; costgraph[13][14] = 4;
            costgraph[13][16] = 4; costgraph[14][13] = 4; costgraph[14][15] = 3; costgraph[15][14] = 3; costgraph[20][19] = 1;
            costgraph[17][16] = 1; costgraph[17][24] = 1; costgraph[17][25] = 1; costgraph[17][18] = 1; costgraph[20][21] = 1;
            costgraph[18][17] = 1; costgraph[18][23] = 1; costgraph[18][26] = 1; costgraph[18][19] = 1; costgraph[24][17] = 1;
            costgraph[19][18] = 1; costgraph[19][22] = 1; costgraph[19][27] = 1; costgraph[19][20] = 1; costgraph[23][18] = 1;
            costgraph[22][19] = 1; costgraph[25][17] = 1; costgraph[26][18] = 1; costgraph[27][19] = 1; costgraph[15][3]  = 4;
            costgraph[10][6] = 7;

            // Direction Graph which will contain the informations of direction between 
            // multiple subsequent Nodes
            // North - 0 East - 1 West - 2 South - 3
            directions[0][1] = 1; directions[0][5] = 3; directions[1][0] = 2; directions[1][2] = 3; directions[1][16] = 1;
            directions[2][1] = 0; directions[2][3] = 3; directions[3][2] = 0; directions[3][4] = 2; directions[3][20] = 1;
            directions[4][3] = 1; directions[4][5] = 2; directions[5][4] = 1; directions[5][0] = 0; directions[3][15] = 3;
            directions[5][6] = 3; directions[6][5] = 0; directions[6][7] = 1; directions[7][6] = 2; directions[6][10] = 3;
            directions[7][8] = 1; directions[8][7] = 2; directions[8][9] = 3; directions[9][8] = 0; directions[8][15] = 0;
            directions[10][9]  = 0; directions[9][10]  = 3; directions[15][8]  = 3; directions[16][1]  = 0; directions[20][3]  = 2;
            directions[10][11] = 1; directions[11][12] = 1; directions[16][17] = 2; directions[16][13] = 3; directions[21][20] = 0;
            directions[11][10] = 2; directions[12][11] = 3; directions[12][13] = 0; directions[13][12] = 3; directions[13][14] = 2;
            directions[13][16] = 0; directions[14][13] = 1; directions[14][15] = 2; directions[15][14] = 1; directions[20][19] = 1;
            directions[17][16] = 1; directions[17][24] = 3; directions[17][25] = 0; directions[17][18] = 2; directions[20][21] = 3;
            directions[18][17] = 1; directions[18][23] = 3; directions[18][26] = 0; directions[18][19] = 2; directions[24][17] = 0;
            directions[19][18] = 1; directions[19][22] = 3; directions[19][27] = 0; directions[19][20] = 2; directions[23][18] = 0;
            directions[22][19] = 0; directions[25][17] = 3; directions[26][18] = 3; directions[27][19] = 3; directions[15][3]  = 0;
            directions[10][6] = 2;
        end
    end
    else if(rxdone == 1 && taskend == 0) begin
        case(present)
            init:
                begin
                    // Variable Re-Initialization
                    // This is important as path planning has to occur multiple times
                    // while bot is traversing in the field in realtime. 
                    complete = 0; count = 1; flag1 = 0; flag2 = 0; clear = 0; reset = 0;
                    startnode = targets[traverseindex]; targetnode = targets[traverseindex+1];
                    if(i < nodes) begin
                        distance[i] = 0; status[i] = 0; turn[i] = 0;
                        prediction[i] = 0; field[i] = 0; nodex[i] = 0;
                        i = i + 1'b1;
                    end
                    else begin 
                        i = 1; j = 0; present = choosetarget;
                    end
                end
            choosetarget:
                begin
                    // If place operation need to be done, then choose appropriate
                    // start and traverseindex node
                    if(place == 1) begin
                        startnode = ptarget; targetnode = itarget[placetrack]; present = validate;
                    end
                    // If Pick operation need to be done, then choose correct startnode
                    else if(pick == 1 && picktrack != 0 && i == 1) startnode = itarget[picktrack-1];
                    if(((siidmax == 1 && traverseindex < 15) || last == 1) && place == 0) begin
                        // If Any Color detected, then the package of respective color
                        // will be picked from warehouse by searching an array
                        if(i < 8) begin
                            if(icolor[picktrack] == packages[i] && flag1 == 0) begin
                                targetnode = warehouse[i]; ptarget = targetnode;
                                flag1 = 0; pick = 1; temp_index = i;
                                if(startnode == 12 || startnode == 13) flag1 = 1;
                            end
                            i = i + 1'b1;
                        end
                        else present = validate;
                    end
                    // If three SI is not identified, then continue traverseindex array
                    else if(siidmax == 0) begin
                        i = 0; tempflag = 1; flag1 = 0;
                        // If the No of patches placed in the arena is less than three,
                        // then start pick operation after traversing all the possible SI Nodes
                        if(ci != 0 && traverseindex == 15 && last == 0) begin
                            pick = 1; present = choosetarget; last = 1; taskend = 0;
                        end
                        else if(pick == 0 && place == 0) traverseindex = traverseindex + 1'b1;
                        // If Max SI's Identified, then task is completed after
                        // placing them appropriately.
                        if(sic == 7) begin
                            taskend = 1; present = init;
                        end
                        else if(next == 0) present = validate;
                        else if(traverseindex < 15) present = validate;
                    end
                    // End of the Run
                    if(next == 1 && traverseindex >= 15 && track == sic && place == 0) begin
                        taskend = 1; rotate = 0; enableem = 0; present = init;
                    end
                end

            validate:
                begin
                    if(complete == 0) begin
                        // If the start or traverseindex node is same or beyond the maximum node,
                        // then mark path planning as completed
                        if(tempflag == 1) begin
                            if(startnode > 28 || targetnode > 28 || startnode == targetnode ||
                            startnode == 0 || targetnode == 0) complete = 1;
                            else begin
                                // Swaping Variables
                                swap = startnode-1'b1; clear = 1;
                                startnode = targetnode-1'b1;
                                targetnode = swap; tempflag = 0;
                                i = 0; j = 0; present = dijkstra;
                            end
                        end
                    end
                    else present = choosetarget;
                end
            dijkstra:
                begin
                    case(algorithm)
                    s0:
                        begin
                            if(tempflag == 0) begin
                                // Exploring Graph
                                if(i<nodes) begin
                                    distance[i]=costgraph[startnode][i];
                                    prediction[i] = startnode; nodesvisited[i] = 0;
                                    i = i + 1'b1;
                                end
                                else begin
                                    algorithm = s1; i = 0; tempflag = 0;
                                    distance[startnode] = 0; nodesvisited[startnode] = 1;
                                end
                            end
                        end
                    s1:
                        begin
                            // Computing Shortest Path for each Node from startNode
                            if(count<nodes) begin
                                if(i == 0 && j == 0) mindist = inf;
                                if(tempflag == 0) begin
                                    if(i<nodes) begin
                                        if((distance[i]<mindist)&&(!nodesvisited[i])) begin
                                            mindist = distance[i]; nextnode = i;
                                        end
                                        i = i + 1'b1;
                                    end
                                    else begin
                                        tempflag = 1; nodesvisited[nextnode] = 1;
                                    end
                                end
                                else begin
                                    if(tempflag == 1) begin
                                        if(j<nodes) begin
                                            if(!nodesvisited[j]) begin
                                                if((mindist+costgraph[nextnode][j])<distance[j]) begin
                                                    distance[j] = mindist + costgraph[nextnode][j];
                                                    prediction[j] = nextnode;
                                                end
                                            end
                                            j = j + 1'b1;
                                        end
                                        else begin
                                            i = 0; j = 0; tempflag = 0; count = count + 1'b1;
                                        end
                                    end
                                end
                            end
                            else begin
                                i = 0; algorithm = s2;
                            end
                        end
                    s2:
                        begin
                            // If predicted Nodes goes beyond the maximum Nodes,
                            // then make path planning to be completed.
                            if(i<nodes) begin
                                if(prediction[i] > 28) complete = 1;
                                i = i + 1'b1;
                            end
                            else begin
                                algorithm = s0; tempflag = 1;
                            end
                        end
                    endcase
                    if(algorithm == s0 && tempflag == 1) begin
                        present = planner; tempflag = 0;
                    end
                end
            planner:
                begin
                    // Get Result
                    // The result will contain the shortest distance and
                    // all the subset of nodes to be traversed.
                    case(plannerx)
                        predict:
                            begin
                                if(tempflag == 0) begin
                                    i = 1; j = 0; k = 0; l = 0; count = 0;
                                    j = targetnode; nodex[0] = targetnode; tempflag = 1;
                                end
                                else if(complete == 0) begin
                                    count = count + 1'b1; l = prediction[j];
                                    nodex[i] = l;
                                    j = prediction[j]; i = i + 1'b1;
                                    if(j < 0 || j > 28 || i > 28 || j == startnode) begin
                                        i = 0; plannerx = compute;
                                    end
                                end
                            end
                        compute:
                            if(i < count) begin
                                // Current and Future Nodes
                                j = nodex[i]; l = nodex[i+1];
                                // Specific Condition for the Node Connected in between two corner Nodes
                                if(targetnode == 0 && j == 0 && diru == 1) botface = 0;
                                else if(targetnode == 0 && j == 0) botface = directions[j][l];
                                // Here Direction of Turn will be obtained which is based on the
                                // directions of the bot traversing node and its subsequent node
                                direct = directions[j][l];
                                if(botface == direct) turn[i] = 1;
                                else if(botface != direct) begin
                                    if(botface == 0) begin
                                        if(direct == 1) turn[i] = 5;
                                        else if(direct == 2) turn[i] = 6;
                                        else if(direct == 3) turn[i] = 7;
                                    end
                                    else if(botface == 1) begin
                                        if(direct == 0) turn[i] = 6;
                                        else if(direct == 2) turn[i] = 7;
                                        else if(direct == 3) turn[i] = 5;
                                    end
                                    else if(botface == 2) begin
                                        if(direct == 0) turn[i] = 5;
                                        else if(direct == 1) turn[i] = 7;
                                        else if(direct == 3) turn[i] = 6;
                                    end
                                    else if(botface == 3) begin
                                        if(direct == 0) turn[i] = 7;
                                        else if(direct == 1) turn[i] = 6;
                                        else if(direct == 2) turn[i] = 5;
                                    end
                                end
                                else turn[i] = 1;
                                // Assigning Status and Field to Appropriate Index of the Array
                                if(pick == 0 && place == 0) begin
                                    status[i] = statusnode[j][l]; field[i] = farmnode[j][l];
                                end
                                if(place == 1 && l == startnode) begin
                                    if((statusnode[j][l] != 0)) turn[count] = 5;
                                    else turn[count] = 6;
                                    turn[count+1] = 4;
                                end
                                // Corner Conditions are Added Here
                                if(j == corner[0] || j == corner[1] || j == corner[2] || j == corner[3]) flag1 = 1;
                                else if(j == corner[4] || j == corner[5] || j == corner[6] || j == corner[7]) flag1 = 1;
                                if(l == corner[0] || l == corner[1] || l == corner[2] || l == corner[3]) flag2 = 1;
                                else if(l == corner[4] || l == corner[5] || l == corner[6] || l == corner[7]) flag2 = 1;
                                // A Node Connected between two edges
                                if(j == 11 && l == 10) flag1 = 0;
                                if(j == 0 && pick == 1) flag1 = 0;
                                // If the Node is in between two Corners then, turn is not updated,
                                // while the direction alone updated.
                                // Updating BotFace if the targetnode(swapped) is one
                                // of the corner node
                                if(flag2 == 1 && l == startnode) tempdir = directions[l][j];
                                // Updating Direction of the Bot
                                if(flag1 == 1 && flag2 == 1 && targetnode != 0) begin
                                    k = prediction[l]; direct = directions[l][k];
                                    if(k == l) direct = directions[j][l];
                                end
                                botface = direct;
                                flag1 = 0; flag2 = 0; i = i + 1'b1;
                            end
                            else begin
                                if(tempdir != 0) botface = 2'b11- tempdir;
                                if(place == 1) begin
                                    botface = deposition[startnode];
                                    if(startnode == 0) diru = 1;
                                end
                                i = 0; plannerx = predict; tempflag = 1;
                                next = 1; present = tracker; tempdir = 0;
                            end
                    endcase
                end
            tracker:
                begin
                    // Map turn array to output register
                    rotate = turn[node];
                    // If reset is high, then all the temporary values stored will
                    // be cleared
                    if (reset == 1) present = tracker;
                    else if(reset == 0 || done == 1) begin
                        if(turn[node+1] == 0 && pick == 1 && place == 0) enableem = 1;
                        if(turn[node] == 4) begin
                            enableem = 0; offind = placetrack; txm3 = 1;
                        end
                        else if(turn[node] == 0 && place == 1) begin
                            place = 0; placetrack = placetrack + 1'b1;
                            track = track + 1'b1; offind = 3;
                            if(placetrack == 3) begin
                                reset = 1; placetrack = 0; picktrack = 0; ptarget = 31;
                            end
                        end
                        else if(turn[node] == 0 && pick == 1) begin
                            place = 1; picktrack = picktrack + 1'b1; txm2 = 1;
                            if(picktrack == 3 || track == sic) pick = 0;
                        end
                        if(turn[node] == 0) present = init;
                        else present = tracker;
                    end
                end
        endcase
        if(once == 1) txm2 = 0;
        else txm3 = 0;
    end
end
endmodule
