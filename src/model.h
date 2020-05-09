#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <cstddef>


// The model class contains data structures to store the OpenStreetMap data
class Model
{
public:
    struct Node {
        double x = 0.f;
        double y = 0.f;
    };
    
    struct Way {
        std::vector<int> nodes;
    };
    
    struct Road {
        enum Type { Invalid, Unclassified, Service, Residential,
            Tertiary, Secondary, Primary, Trunk, Motorway, Footway };
        int way;
        Type type;
    };
    
    struct Railway {
        int way;
    };    
    
    struct Multipolygon {
        std::vector<int> outer;
        std::vector<int> inner;
    };
    
    struct Building : Multipolygon {};
    
    struct Leisure : Multipolygon {};
    
    struct Water : Multipolygon {};
    
    struct Landuse : Multipolygon {
        enum Type { Invalid, Commercial, Construction, Grass, Forest, Industrial, Railway, Residential };
        Type type;
    };
    
    // Model constructor - accepts a vector of bytes representing the xml data
    Model( const std::vector<std::byte> &xml );
    
    // To convert node x and y coordinates from latitude and longitude to distance in meters
    auto MetricScale() const noexcept { return m_MetricScale; }    
    
    // Public functions to return the data stored in the model
    auto &Nodes() const noexcept { return m_Nodes; }
    auto &Ways() const noexcept { return m_Ways; }
    auto &Roads() const noexcept { return m_Roads; }
    auto &Buildings() const noexcept { return m_Buildings; }
    auto &Leisures() const noexcept { return m_Leisures; }
    auto &Waters() const noexcept { return m_Waters; }
    auto &Landuses() const noexcept { return m_Landuses; }
    auto &Railways() const noexcept { return m_Railways; }
    
private:
    // Converts lat/lon data to meters
    void AdjustCoordinates();

    // Takes OSM multipolygon data and stores it in the model
    void BuildRings( Multipolygon &mp );

    // Function that takes in the data and reads it into all of the objects into the model
    void LoadData(const std::vector<std::byte> &xml);
    
    // Model data stored in private vectors. These are vectors of struct objects
    std::vector<Node> m_Nodes;
    std::vector<Way> m_Ways;
    std::vector<Road> m_Roads;
    std::vector<Railway> m_Railways;
    std::vector<Building> m_Buildings;
    std::vector<Leisure> m_Leisures;
    std::vector<Water> m_Waters;
    std::vector<Landuse> m_Landuses;
    
    double m_MinLat = 0.;
    double m_MaxLat = 0.;
    double m_MinLon = 0.;
    double m_MaxLon = 0.;
    double m_MetricScale = 1.f;
};
