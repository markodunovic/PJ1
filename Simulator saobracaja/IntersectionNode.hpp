#pragma once
#include "RoadNode.hpp"
#include <vector>
#include <map>
#include <algorithm>
#include <optional>

class IntersectionNode : public Node
{
private:
    std::map<std::shared_ptr<RoadNode>, std::vector<std::pair<std::shared_ptr<RoadNode>, double>>> connections;
    int maxVehicles;
    int currentVehicles = 0;
    double maxIntersectionSpeed = 50;
    double minIntersectionSpeed = 10;

public:
    IntersectionNode(const std::string &id, int maxVehicles) : Node(id), maxVehicles(maxVehicles) {}

    std::map<std::shared_ptr<RoadNode>, std::vector<std::pair<std::shared_ptr<RoadNode>, double>>> getConnections() const
    {
        return this->connections;
    }

    bool isInterSectionFull() const
    {
        return (this->maxVehicles - this->currentVehicles) <= 0;
    }

    void increaseCurrentVehicles()
    {
        this->currentVehicles++;
    }

    void decreaseCurrentVehicles()
    {
        this->currentVehicles--;
    }

    int getMaxVehicles() const
    {
        return this->maxVehicles;
    }

    void setMaxVehicles(int newMaxVehicles)
    {
        if (newMaxVehicles >= 0)
        {
            this->maxVehicles = newMaxVehicles;
        }
    }

    void addConnection(std::shared_ptr<RoadNode> inRoad, std::shared_ptr<RoadNode> outRoad, double length)
    {
        auto &outRoads = this->connections[inRoad];
        if (std::none_of(outRoads.begin(), outRoads.end(), [outRoad](const std::pair<std::shared_ptr<RoadNode>, double> &p)
                         { return p.first == outRoad; }))
        {
            outRoads.push_back(std::make_pair(outRoad, length));
        }
    }

    bool canTransition(std::shared_ptr<RoadNode> inRoad, std::shared_ptr<RoadNode> outRoad) const
    {
        // find returns an iterator to the element if it is found, otherwise it returns an iterator to the end of the container
        if (this->connections.find(inRoad) != this->connections.end())
        {
            const auto &outRoads = this->connections.at(inRoad); // returns a reference to the mapped value of the element with a key equivalent to k (k is inRoad)
            for (const auto &pair : outRoads)
            {
                if (pair.first == outRoad) // pair.first is the outRoad
                {
                    return true;
                }
            }
        }
        return false;
    }

    std::optional<double> getTransitionLength(std::shared_ptr<RoadNode> inRoad, std::shared_ptr<RoadNode> outRoad) const
    {
        if (this->connections.find(inRoad) != this->connections.end())
        {
            const auto &outRoads = this->connections.at(inRoad);
            for (const auto &pair : outRoads)
            {
                if (pair.first == outRoad)
                {
                    return pair.second; // pair.second is the length
                }
            }
        }
        return std::nullopt; // transition not found
    }

    void updateVehicleCount(int vehiclesNumber)
    {
        if (vehiclesNumber >= 0 && vehiclesNumber <= maxVehicles)
        {
            this->currentVehicles = vehiclesNumber;
        }
    }

    double calculateAverageSpeed() const
    {
        if (this->maxVehicles == 0)
            return this->minIntersectionSpeed;
        double averageSpeed = maxIntersectionSpeed - ((double)currentVehicles / maxVehicles) * (maxIntersectionSpeed - minIntersectionSpeed);
        return std::max(averageSpeed, this->minIntersectionSpeed);
    }

    bool checkAndCalculateTransition(std::shared_ptr<RoadNode> inRoad, std::shared_ptr<RoadNode> outRoad, double &transitionLength, double &averageSpeed) const
    {
        if (this->canTransition(inRoad, outRoad))
        {
            auto transition = this->getTransitionLength(inRoad, outRoad);
            if (transition)
            {
                transitionLength = *transition;
                averageSpeed = this->calculateAverageSpeed();
                return true;
            }
        }
        return false;
    }
};