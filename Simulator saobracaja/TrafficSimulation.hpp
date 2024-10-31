#pragma once
#include "Vehicle.hpp"
#include <random>
#include <chrono>
#include <thread>

class TrafficSimulation
{
private:
    TrafficNetworkGraph network;
    std::vector<Vehicle> vehicles;
    double simulationStepDuration;
    std::vector<double> travelTimes;

public:
    TrafficSimulation(const TrafficNetworkGraph &network, double stepDuration) : network(network), simulationStepDuration(stepDuration) {}

    // Dodavanje vozila u simulaciju
    void addVehicle(const Vehicle &vehicle)
    {
        this->vehicles.push_back(vehicle);
    }

    // Izračunavanje najboljih putanja za sva vozila
    void calculateBestPaths()
    {
        for (auto &vehicle : this->vehicles)
        {
            vehicle.calculateBestPath(network, true);
        }
    }

    // Provera da li su sva vozila stigla do krajnje destinacije
    bool allVehiclesReachedDestination() const
    {
        for (const auto &vehicle : this->vehicles)
        {
            if (vehicle.getCurrentLocation() != vehicle.getDestination())
            {
                return false;
            }
        }
        return true;
    }

    void showSimulationState() const
    {
        std::cout << "Current simulation state:" << std::endl;
        for (const auto &vehicle : this->vehicles)
        {
            std::cout << "Vehicle " << vehicle.getId() << " is at location " << vehicle.getCurrentLocation()->getIdentifier() << "  |  " << vehicle.getRoadLengths().front() << " km." << std::endl;
        }
        std::cout << std::endl;
    }

    void executeSimulationStep()
    {
        for (auto &vehicle : this->vehicles)
        {
            if (vehicle.getCurrentLocation() != vehicle.getDestination())
            {
                if (auto currentNode = std::dynamic_pointer_cast<IntersectionNode>(vehicle.getCurrentLocation()))
                {
                    double averageSpeed = currentNode->calculateAverageSpeed();
                    double distanceToTravel = averageSpeed * this->simulationStepDuration / 3600;
                    std::optional<double> transitionLength;
                    if (auto nextRoadNode = std::dynamic_pointer_cast<RoadNode>(vehicle.getNextNode()))
                    {
                        transitionLength = currentNode->getTransitionLength(vehicle.getInRoad(), nextRoadNode);
                        if ((transitionLength.value() - distanceToTravel) <= 0)
                        {
                            if (auto nextRoadNode = std::dynamic_pointer_cast<RoadNode>(vehicle.getNextNode()))
                            {
                                if (!nextRoadNode->isRoadFull())
                                {
                                    vehicle.move();
                                    currentNode->decreaseCurrentVehicles();
                                }
                            }
                        }
                    }
                }
                else if (auto nextNode = std::dynamic_pointer_cast<RoadNode>(vehicle.getNextNode()))
                {
                    if (!nextNode->isRoadFull())
                    {
                        vehicle.move();
                        nextNode->increaseCurrentVehicles();
                    }
                }
                else if (auto currentNode = std::dynamic_pointer_cast<RoadNode>(vehicle.getCurrentLocation()))
                {
                    double distanceToTravel = std::min(currentNode->getMaxSpeed(), vehicle.getAverageSpeed()) * this->simulationStepDuration / 3600;
                    vehicle.getRoadLengths().front() -= distanceToTravel;
                    if (vehicle.getRoadLengths().front() <= 0)
                    {
                        if (auto nextNode = std::dynamic_pointer_cast<IntersectionNode>(vehicle.getNextNode()))
                        {
                            vehicle.setInRoad(currentNode);
                            if (!nextNode->isInterSectionFull())
                            {
                                vehicle.move();
                                currentNode->decreaseCurrentVehicles();
                                if (vehicle.getRoadLengths().front() < 0.0)
                                {
                                    this->decreaseLengthNextRoadNode(vehicle);
                                }
                            }
                        }
                        // Onda je cvor lokacije, pomjeri ga
                        else
                        {
                            vehicle.move();
                            currentNode->decreaseCurrentVehicles();
                            this->decreaseLengthNextRoadNode(vehicle);
                        }
                    }
                }
            }
        }

        // Pauziranje simulacije za definisano trajanje koraka, ipak 1 sekundu
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(1)));

        // Provera da li su sva vozila stigla do krajnje destinacije
        if (this->allVehiclesReachedDestination())
        {
            for (const auto &vehicle : this->vehicles)
            {
                this->travelTimes.push_back(vehicle.getTravelTime());
            }

            this->showSimulationState();
            std::cout << "All vehicles reached their destinations. Simulation complete." << std::endl;
        }
        else
            std::cout << "Simulation is not complete yet." << std::endl;
    }

    void showAverageTravelTime() const
    {
        double totalTravelTime = 0.0;
        for (const auto &time : travelTimes)
        {
            totalTravelTime += time;
        }
        double averageTravelTime = totalTravelTime / travelTimes.size();
        std::cout << "Average travel time for all vehicles: " << averageTravelTime << " seconds.\n"
                  << std::endl;
    }

    void decreaseLengthNextRoadNode(Vehicle &vehicle)
    {
        if (vehicle.getRoadLengths().size() == 1)
        {
            return;
        }
        if (!vehicle.getRoadLengths().empty())
        {
            double rest = vehicle.getRoadLengths().front();
            vehicle.getRoadLengths().erase(vehicle.getRoadLengths().begin());
            // std::cout << "\n--------------(" << vehicle.getRoadLengths().size() << ")-----------\n";
            if (vehicle.getRoadLengths().size() == 1)
            {
                vehicle.getRoadLengths().front() = 0.0;
                return;
            }
            if (!vehicle.getRoadLengths().empty())
            {
                vehicle.getRoadLengths().front() += rest;
            }
        }
    }
};