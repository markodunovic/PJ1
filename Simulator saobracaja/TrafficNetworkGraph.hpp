#pragma once
#include "IntersectionNode.hpp"
#include <unordered_map>
#include <sstream>
#include <fstream>

class TrafficNetworkGraph
{
private:
    std::unordered_map<std::string, std::shared_ptr<Node>> nodes;

public:
    const std::unordered_map<std::string, std::shared_ptr<Node>> &getNodes() const
    {
        return this->nodes;
    }

    // adding a location to the graph
    void addLocation(const std::string &id)
    {
        auto location = std::make_shared<LocationNode>(id);
        this->nodes[id] = location;
    }

    // adding a road to the graph
    void addRoad(const std::string &id, const std::string &startId, const std::string &endId, double length, double maxSpeed, int maxVehicles)
    {
        auto startNode = this->nodes.find(startId);
        auto endNode = this->nodes.find(endId);
        if (startNode != this->nodes.end() && endNode != this->nodes.end())
        {
            auto road = std::make_shared<RoadNode>(id, startNode->second, endNode->second, length, maxSpeed, maxVehicles);
            this->nodes[id] = road;
        }
    }

    // adding an intersection to the graph
    void addIntersection(const std::string &id, int maxVehicles)
    {
        auto intersection = std::make_shared<IntersectionNode>(id, maxVehicles);
        this->nodes[id] = intersection;
    }

    // connecting roads to an intersection
    void connectRoads(const std::string &intersectionId, const std::string &inRoadId, const std::vector<std::pair<std::string, double>> &outRoadsInfo)
    {
        auto intersectionNode = std::dynamic_pointer_cast<IntersectionNode>(this->nodes[intersectionId]);
        if (intersectionNode)
        {
            for (const auto &[outRoadId, length] : outRoadsInfo)
            {
                auto inRoad = std::dynamic_pointer_cast<RoadNode>(this->nodes[inRoadId]);
                auto outRoad = std::dynamic_pointer_cast<RoadNode>(this->nodes[outRoadId]);
                if (inRoad && outRoad)
                {
                    intersectionNode->addConnection(inRoad, outRoad, length);
                }
            }
        }
    }

    // finding a node by its id
    std::shared_ptr<Node> findNodeById(const std::string &id) const
    {
        auto it = nodes.find(id);
        if (it != nodes.end())
        {
            return it->second;
        }
        return nullptr;
    }

    std::shared_ptr<LocationNode> getLocationNode(const std::string &id) const
    {
        auto it = nodes.find(id);
        if (it != nodes.end())
        {
            return std::dynamic_pointer_cast<LocationNode>(it->second);
        }
        return nullptr;
    }

    void serialize(const std::string &filename) const
    {
        std::ofstream outfile(filename);

        // Prvi red - čvorovi lokacija
        for (const auto &[id, node] : nodes)
        {
            if (std::dynamic_pointer_cast<LocationNode>(node))
            {
                outfile << id << ";";
            }
        }
        outfile << "\n";

        // Drugi red - čvorovi raskrsnica
        for (const auto &[id, node] : nodes)
        {
            if (auto intersection = std::dynamic_pointer_cast<IntersectionNode>(node))
            {
                outfile << "(" << intersection->getIdentifier() << "," << intersection->getMaxVehicles() << ");";
            }
        }
        outfile << "\n";

        // Treći red - čvorovi puteva
        for (const auto &[id, node] : nodes)
        {
            if (auto road = std::dynamic_pointer_cast<RoadNode>(node))
            {
                outfile << "(" << road->getIdentifier() << "," << road->getStartNode()->getIdentifier() << ","
                        << road->getEndNode()->getIdentifier() << "," << road->getLength() << ","
                        << road->getMaxSpeed() << "," << road->getMaxVehicles() << ");";
            }
        }
        outfile << "\n";

        // Četvrti red - veze između puteva na raskrsnicama
        for (const auto &[id, node] : nodes)
        {
            if (auto intersection = std::dynamic_pointer_cast<IntersectionNode>(node))
            {
                const auto &connections = intersection->getConnections();
                for (const auto &[inRoad, outRoads] : connections)
                {
                    outfile << "(" << intersection->getIdentifier() << "," << inRoad->getIdentifier() << ",{";
                    for (const auto &[outRoad, length] : outRoads)
                    {
                        outfile << outRoad->getIdentifier() << "," << length << ",";
                    }
                    outfile.seekp(-1, std::ios_base::end); // Sklanjamo poslednji zarez
                    outfile << "});";
                }
            }
        }
        outfile << "\n";

        outfile.close();
    }

    void deserialize(const std::string &filename)
    {
        std::ifstream infile(filename);
        if (!infile)
        {
            std::cerr << "Error: Unable to open file for reading." << std::endl;
            return;
        }

        std::string line;
        // Prvi red - čvorovi lokacija
        if (std::getline(infile, line))
        {
            std::istringstream iss(line);
            std::string id;
            while (std::getline(iss, id, ';'))
            {
                addLocation(id);
            }
        }

        // Drugi red - čvorovi raskrsnica
        if (std::getline(infile, line))
        {
            std::istringstream iss(line);
            char delimiter;
            while (iss >> delimiter) // Čitamo otvorenu zagradu
            {
                if (delimiter == '(') // Ako je pročitani karakter otvorena zagrada
                {
                    std::string intersectionData;
                    std::getline(iss, intersectionData, ')'); // Čitamo podatke o raskrsnici do zatvorene zagrade
                    intersectionData += ')';                  // Dodajemo zatvorenu zagradu na kraj kako bismo imali cijeli zapis raskrsnice

                    std::istringstream intersection_ss(intersectionData);
                    std::string id, maxVehicles_str;
                    std::getline(intersection_ss, id, ',');              // Čitamo id
                    std::getline(intersection_ss, maxVehicles_str, ','); // Čitamo maksimalni broj vozila
                    maxVehicles_str.pop_back();                          // Uklanjamo zatvorenu zagradu sa kraja
                    int maxVehicles = std::stoi(maxVehicles_str);        // Konvertujemo string u integer
                    addIntersection(id, maxVehicles);                    // Dodajemo raskrsnicu u graf
                }
            }
        }

        // std::cout << "PRIJE TRECEG REDA\n";

        // Treći red - čvorovi puteva
        if (std::getline(infile, line))
        {
            std::istringstream iss(line);
            char delimiter;
            while (iss >> delimiter) // Čitamo otvorenu zagradu
            {
                if (delimiter == '(') // Ako je pročitani karakter otvorena zagrada
                {
                    std::string roadData;
                    std::getline(iss, roadData, ')'); // Čitamo podatke o putu do zatvorene zagrade
                    roadData += ')';                  // Dodajemo zatvorenu zagradu na kraj kako bismo imali cijeli zapis puta

                    std::istringstream road_ss(roadData);
                    std::string id, startId, endId, length_str, maxSpeed_str, maxVehicles_str;
                    std::getline(road_ss, id, ',');                             // Čitamo id
                    std::getline(road_ss, startId, ',');                        // Čitamo startId
                    std::getline(road_ss, endId, ',');                          // Čitamo endId
                    std::getline(road_ss, length_str, ',');                     // Čitamo length
                    double length = std::stod(length_str);                      // Konvertujemo string u double
                    std::getline(road_ss, maxSpeed_str, ',');                   // Čitamo maxSpeed
                    double maxSpeed = std::stod(maxSpeed_str);                  // Konvertujemo string u double
                    std::getline(road_ss, maxVehicles_str, ')');                // Čitamo maxVehicles
                    int maxVehicles = std::stoi(maxVehicles_str);               // Konvertujemo string u integer
                    addRoad(id, startId, endId, length, maxSpeed, maxVehicles); // Dodajemo put u graf
                }
            }
        }

        // std::cout << "PRIJE CETVRTOG REDA\n";

        // Četvrti red - veze između puteva na raskrsnicama
        if (std::getline(infile, line))
        {
            std::istringstream iss(line);
            std::string token;
            while (std::getline(iss, token, ';')) // Razdvajamo elemente po ;
            {
                std::istringstream node_ss(token);
                char delimiter;
                while (node_ss >> delimiter) // Čitamo otvorenu zagradu
                {
                    if (delimiter == '(') // Ako je pročitani karakter otvorena zagrada
                    {
                        std::string intersectionId, inRoadId, outRoadsInfo_str;
                        std::getline(node_ss, intersectionId, ',');   // Čitamo intersectionId
                        std::getline(node_ss, inRoadId, ',');         // Čitamo inRoadId
                        std::getline(node_ss, outRoadsInfo_str, '}'); // Čitamo outRoadsInfo_str
                        // Uklanjamo otvorenu vitičastu zagradu sa početka stringa outRoadsInfo_str
                        outRoadsInfo_str.erase(0, 1); // Uklanjamo prvi karakter koji je '{'

                        // std::cout << intersectionId << " " << inRoadId << " " << outRoadsInfo_str << std::endl;

                        std::istringstream outRoads_ss(outRoadsInfo_str);

                        std::string outRoadInfo;
                        std::vector<std::pair<std::string, double>> outRoadsInfoVec;

                        int count = 0;
                        std::string outRoadId, length_str;
                        while (std::getline(outRoads_ss, outRoadInfo, ','))
                        {
                            if (count % 2 == 0) // Ako je broj paran, tada čitamo outRoadId
                            {
                                outRoadId = outRoadInfo;
                            }
                            else // Ako je broj neparan, tada čitamo dužinu i dodajemo u vektor
                            {
                                length_str = outRoadInfo;
                                // std::cout << outRoadId << "-" << length_str << std::endl;
                                double length = std::stod(length_str);
                                outRoadsInfoVec.push_back(std::make_pair(outRoadId, length));
                            }
                            count++;
                        }
                        connectRoads(intersectionId, inRoadId, outRoadsInfoVec); // Povezujemo puteve na raskrsnici
                    }
                }
            }
        }
        
        infile.close();
    }
};