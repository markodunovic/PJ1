#pragma once
#include "TrafficNetworkGraph.hpp"
#include <limits>
#include <stdexcept>

class Vehicle
{
private:
    std::string id;
    double averageSpeed;
    std::shared_ptr<Node> currentLocation;
    std::shared_ptr<Node> destination;
    std::vector<std::shared_ptr<Node>> bestPath;
    double travelTime;
    std::vector<double> roadLengths;
    std::shared_ptr<RoadNode> inRoad;
    double currentInterSectionLength = 0;

public:
    Vehicle(const std::string &id, double avgSpeed, std::shared_ptr<Node> startLocation, std::shared_ptr<Node> destination)
        : id(id), averageSpeed(avgSpeed), currentLocation(startLocation), destination(destination) {}

    const std::string &getId() const
    {
        return this->id;
    }

    void setInRoad(std::shared_ptr<RoadNode> inRoad)
    {
        this->inRoad = inRoad;
    }

    std::shared_ptr<RoadNode> getInRoad() const
    {
        return this->inRoad;
    }

    double getAverageSpeed() const
    {
        return this->averageSpeed;
    }

    void setAverageSpeed(double speed)
    {
        this->averageSpeed = speed;
    }

    std::shared_ptr<Node> getDestination() const
    {
        return this->destination;
    }

    std::shared_ptr<Node> getCurrentLocation() const
    {
        return this->currentLocation;
    }

    void setCurrentLocation(std::shared_ptr<Node> newLocation)
    {
        this->currentLocation = newLocation;
    }

    std::vector<std::shared_ptr<Node>> getBestPath() const
    {
        return this->bestPath;
    }

    void calculateBestPath(TrafficNetworkGraph &graph, bool minimizeTime)
    {
        const auto &nodes = graph.getNodes();
        std::unordered_map<std::string, double> values;
        std::unordered_map<std::string, std::shared_ptr<Node>> predecessors;

        for (const auto &nodePair : nodes)
        {
            values[nodePair.first] = std::numeric_limits<double>::infinity();
        }
        values[currentLocation->getIdentifier()] = 0;

        for (size_t i = 0; i < nodes.size() - 1; ++i)
        {
            for (const auto &nodePair : nodes)
            {
                auto node = nodePair.second;
                if (auto roadNode = std::dynamic_pointer_cast<RoadNode>(node))
                {
                    double roadValue = 0.0;
                    if (minimizeTime)
                    {
                        double speed = std::min(roadNode->getMaxSpeed(), this->averageSpeed);
                        roadValue = roadNode->getLength() / speed;
                    }
                    else
                    {
                        roadValue = roadNode->getLength();
                    }

                    auto startNode = roadNode->getStartNode();
                    auto endNode = roadNode->getEndNode();
                    if (values[startNode->getIdentifier()] + roadValue < values[endNode->getIdentifier()])
                    {
                        this->roadLengths.push_back(roadNode->getLength());
                        values[endNode->getIdentifier()] = values[startNode->getIdentifier()] + roadValue;
                        predecessors[endNode->getIdentifier()] = roadNode; // Save the road as a predecessor
                    }
                }
            }
        }

        this->roadLengths[this->roadLengths.size() - 1] = 0.0;

        /*
        for (const double &temp : this->roadLengths)
        {
            std::cout << this->id << " " << temp << std::endl;
        }
        std::cout << std::endl;
        */

        // Reconstruct the best path
        bestPath.clear();
        std::shared_ptr<Node> at = destination;
        double totalDistance = 0.0;
        double totalTime = 0.0;
        while (at != nullptr && predecessors.find(at->getIdentifier()) != predecessors.end())
        {
            auto road = std::dynamic_pointer_cast<RoadNode>(predecessors[at->getIdentifier()]);
            bestPath.insert(bestPath.begin(), at);
            if (road)
            {
                bestPath.insert(bestPath.begin(), road); // Add the road to the path
                at = road->getStartNode();
                if (!minimizeTime)
                {
                    totalDistance += road->getLength(); // Add the road length only if minimizing distance
                }
                else
                {
                    double speed = std::min(road->getMaxSpeed(), averageSpeed);
                    double time = road->getLength() / speed; // Calculate time for this road
                    totalTime += time;                       // Add the time for this road
                    totalDistance += road->getLength();      // Add the distance for time minimization as well
                }
            }
        }
        this->travelTime = totalTime * 3600;
        bestPath.insert(bestPath.begin(), currentLocation); // Add the start location at the beginning of the path
        /*
        if (!minimizeTime)
        {
            std::cout << "Vozilo " << id << " je preslo put od " << totalDistance << " km." << std::endl;
        }
        else
        {
            std::cout << "Vozilo " << id << " je preslo put od " << totalDistance << " km za " << totalTime << " sati." << std::endl;
        }
        */
    }

    void move()
    {
        if (bestPath.empty())
        {
            throw std::runtime_error("Best path not calculated or no path available.");
        }

        // Remove the current location (the first node in the path)
        bestPath.erase(bestPath.begin());

        if (!bestPath.empty())
        {
            currentLocation = bestPath.front(); // Update the current location to the next node in the path
        }
    }

    double getTravelTime() const
    {
        return this->travelTime;
    }

    std::shared_ptr<Node> getNextNode() const
    {
        return *(this->bestPath.begin() + 1);
    }

    std::vector<double> &getRoadLengths()
    {
        return this->roadLengths;
    }

    std::vector<double> getRoadLengths() const
    {
        return this->roadLengths;
    }

    void printVector() const
    {
        for (const double &temp : this->roadLengths)
        {
            std::cout << temp << std::endl;
        }
    }
};
