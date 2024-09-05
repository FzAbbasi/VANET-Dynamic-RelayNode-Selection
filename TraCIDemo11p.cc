//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/modules/messages/my_msg_m.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>



using namespace veins;

Define_Module(veins::TraCIDemo11p);

void TraCIDemo11p::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        unique_Id = 0;
        max_degree=0;
        max_dist= 0.0;
        lastUpdatedTime = -1;
       // scheduleAt(simTime() + uniform(0, 40) / 1000.0, new cMessage("Timer"));
        scheduleAt(simTime() + 4, new cMessage("Timer"));
    }
}


void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    if (strcmp(msg->getName(), "Timer") == 0) {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        TraCIMobility* mobility = TraCIMobilityAccess().get(getParentModule());
        bsm->setPos_Data(mobility->getPositionAt(simTime()));
        //bsm->setId_Data(unique_Id++); // Assign a unique ID
        bsm->setId_Data(mobility->getId());
        bsm->setTimestamp(simTime());
        sendDown(bsm);
        scheduleAt(simTime() + uniform(1, 4), msg);
    }else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}





void TraCIDemo11p::handleLowerMsg(cMessage* msg) {
    if (my_msg* message = dynamic_cast<my_msg*>(msg)) {
            onMessageReceived(message);
        }else {
            // Handle other message types
            DemoBaseApplLayer::handleLowerMsg(msg);
        }
}


void TraCIDemo11p::onBSM(DemoSafetyMessage* bsm) {
    adjacency_List.clear();
    currentGrapgh.clear();
    simtime_t message_Timestamp = bsm->getTimestamp();
    lastUpdatedTime = message_Timestamp;
    selecting_Relay_Nodes(bsm); // Select relay nodes for the data
}



void TraCIDemo11p::selecting_Relay_Nodes(DemoSafetyMessage* bsm) {
    //finding one hop nodes from current nod
    position_current_node = bsm->getPos_Data();
    ID_current_node = bsm->getId_Data();
    std::vector<int> one_hop_Neighbors = adjacency_node(ID_current_node,position_current_node);
    if (one_hop_Neighbors.size()>1){
        for (int neighbor : one_hop_Neighbors) {
            double Angle = calculateAngle(neighbor, position_current_node);
            int Region = Regions(Angle);
            double degreeCentrality = calculateDegree(neighbor);
            degree_vect.push_back(degreeCentrality);
            double ClosenessCentrality = calculateClosenessCentrality(neighbor);
            //checking the neighbor is a cut vertex
            bool isCutVertex = checkCutVertex(neighbor);
            //Euclidean distance formula
            double distance = calculateDistance(neighbor, position_current_node, ID_current_node);
            dest_vect.push_back(distance);
            //double pathEntropy = calculateCausalPathEntropy(neighbor,position_current_node); // Calculate causal path entropy
            //finding max degree and max distance
            for (int i = 0; i < degree_vect.size(); i++) {
                if (degree_vect[i] > max_degree) {
                    max_degree = degree_vect[i];
                }
                if (dest_vect[i]>max_dist){
                    max_dist = dest_vect[i];
                }
            }
            // Calculate rank considering Closeness centrality and path entropy
            double rank = calculateRank(degreeCentrality, ClosenessCentrality, isCutVertex, distance, max_degree, max_dist);

            if (Region >= 0 && Region < 4) {
                relayNodes[Region].push_back(neighbor);
                rank_relayNodes[Region].push_back(rank);
            }
        }
        //finding the best relay node among the selected relay nodes for each of the four regions
        for (int i = 0; i < 4; ++i) {
            int bestNode = -1;
            double highestRank = -1;
            const auto& regionRelayNodes = relayNodes[i];
            const auto& regionRankRelayNodes = rank_relayNodes[i];

            for (size_t j = 0; j < regionRelayNodes.size(); ++j) {
                double rank = regionRankRelayNodes[j];
                if (rank > highestRank) {
                    highestRank = rank;
                    bestNode = regionRelayNodes[j];
                }
            }
        }
        send_msg(bestNode);// Send the data to the selected relay nodes
    }else{
        scheduleAt(simTime() + 5, new cMessage("Timer"));
    }
}



std::vector<int> TraCIDemo11p::adjacency_node(int Id, Coord Position) {
    //all vehicles
    std::map<std::string, cModule*> availableCars = mobility->getManager()->getManagedHosts();
    std::map<std::string, cModule*>::iterator it;
    for (it = availableCars.begin(); it != availableCars.end(); it++){
        TraCIMobility* mobility = TraCIMobilityAccess().get(it->second);
        //finding one-hop nodes
        int current_Id = mobility->getId();
        Coord current_pos = mobility->getPositionAt(simTime());
        if (current_Id != Id) {
            double distance = Position.distance(current_pos);
            if (distance < 100.0) {
                currentGrapgh.push_back(current_Id);
                adjacency_List[Id].push_back(current_Id); // Add neighbor ID to the current vehicle's adjacency list
                }
            }
        }
    // Store the current snapshot
    return currentGrapgh;
}





std::vector<int> TraCIDemo11p::getNeighbors(int ID, const Coord& POS){
    neighborsId.clear();
    //all vehicle in simulation
    std::map<std::string, cModule*> availableCars = mobility->getManager()->getManagedHosts();
    std::map<std::string, cModule*>::iterator it;
    for (it = availableCars.begin(); it != availableCars.end(); it++){
        TraCIMobility* mobility = TraCIMobilityAccess().get(it->second);

        if (mobility->getId() != ID) {
            Coord neighborpos = mobility->getPositionAt(simTime());
            double distance = POS.distance(neighborpos);

            if (distance <= 100.0) {
                neighborsId.push_back(mobility->getId());
            }
        }
    }
    return neighborsId;
}



double TraCIDemo11p::calculateAngle(int neighbor, Coord position) {
    //findinf position of neighbor
    Coord neighborPos = neighbor_pos(neighbor);
    //atan2(y, x), it is a mathematical function that computes the arctangent
    double x = 3.14159; // Use M_PI for better precision
    return atan2(neighborPos.y - position.y, neighborPos.x - position.x) * 180 / x;
}


int TraCIDemo11p::Regions(double Angle) {
    if (Angle > 45 && Angle <= 135)
        return 0;
    if (Angle > 135 && Angle <= 225)
        return 1;
    if (Angle > 225 && Angle <= 315)
        return 2;
    return 3;
}


double TraCIDemo11p::calculateDegree(int neighbor) {
    neighborPos = neighbor_pos(neighbor);
    // one-hop neighbors
    std::vector<int> neighbors = getNeighbors(neighbor,neighborPos);
    //size() gives the number of elements in the vector, which corresponds to the number of one-hop neighbors
    return neighbors.size();
}
double TraCIDemo11p::calculateClosenessCentrality(int neighbor) {
    neighborPos = neighbor_pos(neighbor);
    // Get all one-hop neighbors
        std::vector<int> neighbors = getNeighbors(neighbor,neighborPos);
        std::set<int> uniqueNeighbors(neighbors.begin(), neighbors.end()); // Use a set to avoid duplicates
        //Assuming closness=0 , it hasn't been determined how many shortest paths pass through this node
        closness = 0.0;

        for (int source : uniqueNeighbors) {
            for (int target : uniqueNeighbors) {
                if (source != target) {
                    // Calculate the number of shortest paths from source to target that pass through the neighbor
                    int paths_Neighbor = countPathsThroughNode(source, target, neighbor);
                    // Calculate the total number of shortest paths from source to target
                    int sum_Paths = countTotalShortestPaths(source, target);
                    if (sum_Paths > 0) {
                        // Increment betweenness centrality based on the fraction of paths that pass through the neighbor
                        closness += static_cast<double>(paths_Neighbor) / sum_Paths;
                    }
                }
            }
        }
        return closness;
}



int TraCIDemo11p::countPathsThroughNode(int source, int target, int id_node) {
    std::cout<<"this is countPathsThroughNode"<<std::endl;
    std::map<int, int> pathCount; // Map to store the number of paths to each node
    std::map<int, std::set<int>> predecessors; // To track predecessors for path reconstruction
    std::queue<int> BFS_Queue; // BFS queue
    std::set<int> visited_Nodes; // To track visited nodes

    // Initialize BFS
    BFS_Queue.push(source);
    visited_Nodes.insert(source);
    pathCount[source] = 1; // There's one way to reach the source: starting there

    // finding all shortest paths by BFS
    while (!BFS_Queue.empty()) {
        int current = BFS_Queue.front();
        BFS_Queue.pop();

        Coord currentPos = neighbor_pos(current);

        std::vector<int> neighbors = getNeighbors(current,currentPos);

        for (int neighbor : neighbors) {
            if (visited_Nodes.find(neighbor) == visited_Nodes.end()) {
                visited_Nodes.insert(neighbor);
                BFS_Queue.push(neighbor);
            }

            // If this is the first time visiting the neighbor, initialize its path count
            if (pathCount.find(neighbor) == pathCount.end()) {
                pathCount[neighbor] = 0;
            }

            // If the current path is the shortest, update the path count
            pathCount[neighbor] += pathCount[current];
            predecessors[neighbor].insert(current); // Track the predecessor for path reconstruction
        }
    }

    // Count the number of paths that pass through the specified node
    int pathsThroughNode = 0;

    // If the target is reachable, count paths through the specified node
    if (pathCount.find(target) != pathCount.end()) {
        // Check if the node is a predecessor of the target
        for (int pred : predecessors[target]) {
            if (pred == id_node) {
                pathsThroughNode += pathCount[pred]; // Add paths that go through the node
            }
        }
    }

    return pathsThroughNode; // Return the count of paths through the specified node
}



int TraCIDemo11p::countTotalShortestPaths(int source, int target) {


    std::cout<<"this is countTotalShortestPaths"<<std::endl;
    std::map<int, int> pathCount; // Map to store the number of paths to each node
    std::queue<int> BFS_Queue; // BFS queue
    std::set<int> visited_Nodes; // To track visited nodes

    // Initialize BFS
    BFS_Queue.push(source);
    visited_Nodes.insert(source);
    pathCount[source] = 1; // There's one way to reach the source: starting there

    // BFS to find all shortest paths
    while (!BFS_Queue.empty()) {
        int current = BFS_Queue.front();
        BFS_Queue.pop();

        Coord currentPos = neighbor_pos(current);

        std::vector<int> neighbors = getNeighbors(current,currentPos);

        for (int neighbor : neighbors) {
            if (visited_Nodes.find(neighbor) == visited_Nodes.end()) {
                visited_Nodes.insert(neighbor);
                BFS_Queue.push(neighbor);
            }

            // If this is the first time visiting the neighbor, initialize its path count
            if (pathCount.find(neighbor) == pathCount.end()) {
                pathCount[neighbor] = 0;
            }

            // If the current path is the shortest, update the path count
            pathCount[neighbor] += pathCount[current];
        }
    }

    // Return the total number of paths to the target
    return pathCount[target]; // Return the count of total paths to the target
}


Coord TraCIDemo11p::neighbor_pos(int neighbor){
    //all vehicle in simulation
    std::map<std::string, cModule*> availableCars = mobility->getManager()->getManagedHosts();
    std::map<std::string, cModule*>::iterator it;
    for (it = availableCars.begin(); it != availableCars.end(); it++){
        TraCIMobility* mobility = TraCIMobilityAccess().get(it->second);

        if (mobility->getId() == neighbor) {
            neighborPos = mobility->getPositionAt(simTime());
        }
    }
    return neighborPos;
}


bool TraCIDemo11p::checkCutVertex(int neighbor) {

    std::cout<<"size adjust"<<adjacency_List.size()<<std::endl;
    // Ensure the neighbor is a valid index
    if (neighbor < 0) {
        std::cerr << "Error: Invalid neighbor index: " << neighbor << std::endl;
        return false;
    }

    // Store the discovery time of each vertex during the DFS traversal
    std::vector<int> discovery(adjacency_List.size(), -1);
    std::vector<int> low(adjacency_List.size(), -1);
    std::set<int> cutVertices;
    int timer = 0;
    depth_First_Search(neighbor, -1, discovery, low, cutVertices, timer);
    // Check if the neighbor is a cut vertex
    return cutVertices.count(neighbor) > 0;
}




void TraCIDemo11p::depth_First_Search(int node, int parent, std::vector<int>& discovery,
                                     std::vector<int>& low, std::set<int>& cutVertices, int& timer) {
    // Ensure the node is a valid index
    if (node < 0 || node >= adjacency_List.size()) {
        std::cerr << "Error: Invalid node index: " << node << std::endl;
        return;
    }

    // Initialize discovery time and low value
    discovery[node] = low[node] = timer++;
    int childCount = 0; // Count of children in DFS tree

    // Explore all neighbors of the current node
    for (int neighbor : adjacency_List[node]) { // Access the vector of neighbors for the current node
        if (discovery[neighbor] == -1) { // If the neighbor is not visited
            childCount++;
            // Recursively visit the neighbor
            depth_First_Search(neighbor, node, discovery, low, cutVertices, timer);
            // Update low value of the current node
            low[node] = std::min(low[node], low[neighbor]);

            // Check if the current node is a cut vertex
            if (low[neighbor] >= discovery[node] && parent != -1) {
                cutVertices.insert(node);
            }
        } else if (neighbor != parent) { // Update low value for back edges
            low[node] = std::min(low[node], discovery[neighbor]);
        }
    }
    // Special case for root node
    if (parent == -1 && childCount > 1) {
        cutVertices.insert(node);
    }
}


double TraCIDemo11p::calculateDistance(int neighbor , const Coord& POS , int ID) {
    // Get the position of the neighbor node
    Coord neighborPosition = neighbor_pos(neighbor);
    // Calculate the Euclidean distance
    double distance = std::sqrt(std::pow(POS.x - neighborPosition.x, 2) +
                                 std::pow(POS.y - neighborPosition.y, 2));
    return distance;
}


double TraCIDemo11p::calculateRank(double degreeCentrality, double betweennessCentrality, bool isCutVertex, double distance, int maxDegreeCentrality , double maxDistance ) {
    // Calculate the rank based on centrality and distance
    double beta = 0.5; // Weighting factor
    return beta * (isCutVertex ? 1 : 0) + (1 - beta) * (degreeCentrality / maxDegreeCentrality) + (distance / maxDistance);
}



void TraCIDemo11p::send_msg(int id_destination) {
    // Create a new message of type my_msg
    my_msg* message = new my_msg();

    // Check if message creation was successful
    if (!message) {

        std::cerr << "Error: Failed to create message." << std::endl;
        return; // Exit if message creation failed
    }

    // Set the fields of the message
    message->setMsgId(id_destination); // Set the message ID to the destination ID
    message->setDest_Address(dest_Address); // Set the destination address

    std::cout << "Sending message to ID: " << dest_Address << std::endl;

    // Send the message with a slight delay
    sendDelayedDown(message, 0.001); // Send with a slight delay
}


void TraCIDemo11p::onMessageReceived(my_msg* message){

    std::cout<<"message recieved"<<std::endl;
    //implement ....
}
