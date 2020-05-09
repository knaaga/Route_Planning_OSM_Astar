#include "route_model.h"
#include <iostream>

// RouteModel constructor definition
// The Model constructor is called here. When this happens, a collection of Model::Node objects are created. However,
// in order to perform the A* search, RouteModel::Node objects are needed
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {

    // Create RouteModel nodes.
    // The for loop loops over the vector of Model::Node given by this->Nodes(). The Nodes() function returns the vector
    // m_nodes of Model::Node objects (refer to the model.h file)
    // For every one of the Model::Node objects, we create a RouteModel::Node object and push it into a vector called m_nodes
    // This is done by calling the RouteModel::Node constructor (remember constructor call initializes the object)
    // The constructor has three arguments - a counter, a pointer to the RouteModel object (this pointer) and the Model::Node
    // object
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}

// Roads are composed of nodes. We want to be know which road a node belong to
// The unordered map stores key-value pairs. The key will be the index. The value will be a vector of road pointers
// In the outer loop, we iterate over each road
// If the road is a footway, we do not want the path to be drawn on it
// Each road has the index of the way it belongs to as an attribute (road.way)
// Each element in the vector returned by Ways() has a vector of nodes that belong to it as an attribute
// If the hashmap does not have a value for the key node_idx, we create an empty vector as a value
// We then push the reference to road into the values vector corresponding to each key
void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}


// The FindNeighbor method returns a pointer to a node RouteModel::Node object
// parent_model is a pointer to the route model
// Algorithm is similar to that used to finding a minimum in an array
// If closest_node is still equal to a null_ptr, we initialize it
// Essentially, this function returns a pointer to the node which is closest to the current node
// The node_indices is a collection of nodes on a particular road (see calling function FIndNeighbors())
RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

// Each node can be a part of multiple roads
// Each of these roads have a node that is closest to the current node. Therefore, there can be multiple possible directions
// to travel
// This function populates the vector of neighboring nodes
// Remember ways is a collection of nodes
// Once we have a road, we can get a vector containing all other node indices on that road using road->way
void RouteModel::Node::FindNeighbors() {
    for (auto &road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


// Ther user defined coordinates may not correspond to any given node on the map
// Therefore, we need to find the nodes that are closest to the starting and ending coordinates given by the user
// Many nodes in the OSM data are on closed paths or may be isolated. So, nodes on footways should be avoided
// input.distance(SNodes()[node_idx] finds the distance between input and SNodes()[node_idx]
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                dist = input.distance(SNodes()[node_idx]);
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}