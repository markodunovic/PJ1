#pragma once
#include "Node.hpp"

class LocationNode : public Node
{
public:
    LocationNode(const std::string &id) : Node(id) {}
};