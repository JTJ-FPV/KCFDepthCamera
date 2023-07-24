#pragma once

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

namespace DBSCAN
{

typedef struct Point_
{
    float x, y, z;  // X, Y, Z position
    int index_ellipse;
    int clusterID;  // clustered ID
}Point;

class DBSCAN {
public:    
    DBSCAN(unsigned int minPts, float eps, std::vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }

    DBSCAN(const unsigned int minPts, const float eps){
        m_minPoints = minPts;
        m_epsilon = eps;
        // m_points = points;
        // m_pointSize = points.size();
    }

    DBSCAN(DBSCAN &d){
        m_minPoints = d.getMinimumClusterSize();
        m_epsilon = d.getEpsilonSize();
        m_points = d.m_points;
        m_pointSize = d.m_points.size();
    }
    ~DBSCAN(){}

    void setPoint(std::vector<Point> points)
    {
        m_points = points;
        m_pointSize = points.size();
    }

    int run();
    std::vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    unsigned int getClusterPoint() {return num_cluster;}

    
public:
    std::vector<Point> m_points;
    
private:    
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
    unsigned int num_cluster;
};

}


