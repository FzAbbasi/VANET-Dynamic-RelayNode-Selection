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

#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/modules/messages/my_msg_m.h"
#include "veins/base/utils/SimpleAddress.h"
#include <vector>
#include <map>
#include <string>
#include <unordered_map>


namespace veins {

constexpr LAddress::L2Type L2BROADCAST = 0xFFFFFFFF;


class VEINS_API TraCIDemo11p : public DemoBaseApplLayer {


public:
    int unique_Id;
    Coord position_current_node;
    int ID_current_node;
    int bestNode = -10;
    double highestRank;
    int max_degree;
    double max_dist;
    simtime_t lastUpdatedTime;
    Coord neighborPos;
    double closness;
   LAddress::L2Type dest_Address;



    std::vector<int> neighborsId;
    std::unordered_map<int, std::vector<int>> adjacency_List;
    std::vector<int> currentGrapgh;
    std::vector<double> degree_vect;
    std::vector<double> dest_vect;
    std::vector<int> relayNodes[4]; // For four zones
    std::vector<double> rank_relayNodes[4];




public:

    void initialize(int stage) override;
    void handleSelfMsg(cMessage* msg) override;
    void handleLowerMsg(cMessage* msg) override;
    void onBSM(DemoSafetyMessage* bsm) override;


    void selecting_Relay_Nodes(DemoSafetyMessage* bsm);
    std::vector<int> adjacency_node(int Id, Coord Position);
    std::vector<int> getNeighbors(int ID, const Coord& POS);
    double calculateAngle(int neighbor, Coord position);
    int Regions(double Angle);
    double calculateDegree(int neighbor);
    double calculateClosenessCentrality(int neighbor);
    int countPathsThroughNode(int source, int target, int node);
    int countTotalShortestPaths(int source, int target);
    Coord neighbor_pos(int neighbor);
    bool checkCutVertex(int neighbor);
    void depth_First_Search(int node, int parent, std::vector<int>& discovery,
                                             std::vector<int>& low, std::set<int>& cutVertices, int& timer);
    double calculateDistance(int neighbor,const Coord& POS, int ID);
    double calculateRank(double degreeCentrality, double betweennessCentrality,
            bool isCutVertex, double distance, int maxDegreeCentrality , double maxDistance);
    void send_msg(int bestNode);
    void onMessageReceived(my_msg* message);




};

} // namespace veins
