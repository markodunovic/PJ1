#pragma once
#include <iostream>
#include <string>

class Node
{
protected:
    std::string identifier;

public:
    Node(const std::string &id) : identifier(id) {}
    virtual ~Node() {}

    std::string getIdentifier() const
    {
        return this->identifier;
    }
};