#pragma once
#include "LocationNode.hpp"
#include <memory>

class RoadNode : public Node
{
private:
    std::shared_ptr<Node> start;
    std::shared_ptr<Node> end;
    double length;
    double maxSpeed;
    int maxVehicles;
    int currentVehicles = 0;

public:
    RoadNode(const std::string &id, std::shared_ptr<Node> start, std::shared_ptr<Node> end, double length, double maxSpeed, int maxVehicles)
        : Node(id), start(start), end(end), length(length), maxSpeed(maxSpeed), maxVehicles(maxVehicles) {}
    
    std::shared_ptr<Node> getStartNode() const
    {
        return this->start;
    }

    std::shared_ptr<Node> getEndNode() const
    {
        return this->end;
    }

    bool isRoadFull() const
    {
        return (this->maxVehicles - this->currentVehicles) <= 0;
    }

    double getLength() const
    {
        return this->length;
    }

    double getMaxSpeed() const
    {
        return this->maxSpeed;
    }

    int getMaxVehicles() const
    {
        return this->maxVehicles;
    }

    void decreaseLength(double size)
    {
        this->length = this->length - size;
    }

    void decreaseCurrentVehicles()
    {
        this->currentVehicles--;
    }

    void increaseCurrentVehicles()
    {
        this->currentVehicles++;
    }
};