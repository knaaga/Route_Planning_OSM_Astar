#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

// The RouteModel class extends the model class
// The Node class extends the Node struct
class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;

        void FindNeighbors();

        // Here, x refers to this->x
        // The x and y values are from the Model::Node class
        // const because function should not modify the object being passes

        float distance(Node other) const {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        // Default Node constructor
        Node(){}

        // Overloaded constructor with initializer list
        // Takes in an index, a pointer to a RouteModel object and a Node struct object
        // index and parent_model are private variables of the Node class
        // Note that the Model::Node constructor is called from the initializer list - this initializes the struct
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };

    // RouteModel constructor declaration
    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float x, float y);
    auto &SNodes() { return m_Nodes; }

    // path holds the path obtained from A* search
    std::vector<Node> path;
    
  private:
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    std::vector<Node> m_Nodes;

};

#endif
