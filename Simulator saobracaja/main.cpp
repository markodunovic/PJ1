#include "TrafficSimulation.hpp"

int main()
{
    // Initialize the traffic network
    TrafficNetworkGraph trafficNetwork;

    // Adding locations
    trafficNetwork.addLocation("A"); // 1
    trafficNetwork.addLocation("B"); // 5
    trafficNetwork.addLocation("C"); // 8

    // Adding intersections
    trafficNetwork.addIntersection("MI", 50); // 3
    trafficNetwork.addIntersection("SI", 30); // 7

    // Adding roads
    trafficNetwork.addRoad("0", "MI", "A", 0.5, 60, 5);
    trafficNetwork.addRoad("2", "A", "MI", 0.3, 80, 5);
    trafficNetwork.addRoad("4", "MI", "B", 0.8, 70, 6);
    trafficNetwork.addRoad("6", "B", "SI", 0.6, 100, 2);
    trafficNetwork.addRoad("9", "MI", "C", 0.2, 100, 2);
    trafficNetwork.addRoad("10", "C", "SI", 0.7, 90, 4);
    trafficNetwork.addRoad("11", "SI", "MI", 0.4, 100, 2);

    // Connecting roads to intersections
    trafficNetwork.connectRoads("MI", "2", {{"0", 0.01}, {"4", 0.01}, {"9", 0.01}});
    trafficNetwork.connectRoads("MI", "11", {{"0", 0.01}, {"4", 0.01}, {"9", 0.01}});
    trafficNetwork.connectRoads("SI", "6", {{"11", 0.01}});
    trafficNetwork.connectRoads("SI", "10", {{"11", 0.01}});

    // Serializing and deserializing the traffic network
    trafficNetwork.serialize("traffic_network.txt");

    TrafficNetworkGraph deserializeTestGraph;
    deserializeTestGraph.deserialize("traffic_network.txt");
    deserializeTestGraph.serialize("deserializeTest.txt");

    
    // ---------------Vehicle---------------


    // Creating a vehicle
    auto startLocation = trafficNetwork.getLocationNode("B");
    auto destination = trafficNetwork.getLocationNode("C");
    Vehicle vehicle("V1", 60, startLocation, destination);

    // Calculating the best path, true for minimizing time, false for minimizing distance
    std::cout << "\nDa li zelite da minimizujete vreme (1) ili udaljenost (0)?" << std::endl;
    bool minimizeTime;
    std::cin >> minimizeTime;

    vehicle.calculateBestPath(trafficNetwork, minimizeTime);

    std::cout << std::endl;

    // Printting the best path
    std::cout << "Best path for Vehicle " << vehicle.getId() << ": ";
    for (const auto &node : vehicle.getBestPath())
    {
        std::cout << node->getIdentifier() << " -> ";
    }
    std::cout << "END" << std::endl;

    // Simulating the vehicle movement
    std::cout << "\n---POCETAK SIMULACIJE KRETANJA VOZILA---\n";
    while (vehicle.getBestPath().size() > 1)
    {
        vehicle.move();
        std::cout << vehicle.getId() << " moved to " << vehicle.getCurrentLocation()->getIdentifier() << std::endl;
    }
    std::cout << "---KRAJ SIMULACIJE KRETANJA VOZILA---\n\n\n";

    
    // ---------------TrafficSimulation---------------

    
    auto startLocation2 = trafficNetwork.getLocationNode("B");
    auto endLocation2 = trafficNetwork.getLocationNode("C");
    Vehicle vehicle2("V2", 110, startLocation2, endLocation2);

    auto startLocation3 = trafficNetwork.getLocationNode("B");
    auto endLocation3 = trafficNetwork.getLocationNode("C");
    Vehicle vehicle3("V3", 50, startLocation3, endLocation3);

    auto startLocation4 = trafficNetwork.getLocationNode("B");
    auto endLocation4 = trafficNetwork.getLocationNode("C");
    Vehicle vehicle4("V4", 100, startLocation4, endLocation4);

    TrafficSimulation simulation(trafficNetwork, 5);
    simulation.addVehicle(vehicle2);
    simulation.addVehicle(vehicle3);
    simulation.addVehicle(vehicle4);

    simulation.calculateBestPaths();

    
    std::cout << "-----Start state:-----\n\n";
    simulation.showSimulationState();
    std::cout << "\n-----Simulation:-----\n\n";
    while (!simulation.allVehiclesReachedDestination())
    {
        simulation.executeSimulationStep();
        simulation.showSimulationState();
    }
    simulation.showAverageTravelTime();
    

    return 0;
}
