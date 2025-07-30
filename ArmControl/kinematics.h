/*#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <limits>
#include <iostream>
#include <vector>
#include <cmath>

const double PI = 3.14159265358979323846;
const double L = 49.5; // Segment length

// Trigonometric tables
constexpr int TABLE_SIZE = 360;
extern double SIN_TABLE[TABLE_SIZE];
extern double COS_TABLE[TABLE_SIZE];

struct AnglesAndError {
    double theta1;
    double theta2;
    double theta3;
    double error; // Distance between obtained position and target position
};

struct Point {
    double x;
    double z;
};

/*
    DESCRIPTION: Initializes the trigonometric tables
    INPUT: N/A
    ERRORS: N/A
*/
void initTrigTables();

/*
    DESCRIPTION: Normalize the angle to [0, 360)
    INPUT: Angle in degrees
    ERRORS: N/A
*/
inline int normalizeAngle(int deg);

/*
    DESCRIPTION: Calculates the position (x, z) where the end-effector arrives considering the orientation and the 3 angles received
    INPUT: 
        - theta1: Angle between Segment 1 and the rotating base
        - theta2: Angle between Segment 1 and Segment 2
        - theta3: Angle between Segment 2 and Segment 3
    ERRORS: N/A
*/
Point forwardKinematics(double theta1, double theta2, double theta3);

// DESCRIPTION: Class for storing and searching precomputed data
class InverseKinematicsSolver {
public:
    struct AngleSet {
        double theta1, theta2, theta3;
        Point pos;
    };


    /*
        DESCRIPTION: Calls precompute and buildGrid.
        INPUT: 
            - cellSize
            - step: The rate of incraese in degrees
        ERRORS: N/A
    */
    InverseKinematicsSolver(double cellSize = 2.0, int step = 2);

    /*
        DESCRIPTION: Searches the grid and its neighboring cells to find the precomputed angle combination that most closely matches the target position
        INPUT: 
            - targetX: The position on the x-axis of the point we want to reach
            - targetY: The position on the z-axis of the point we want to reach
        ERRORS: N/A
    */
    AnglesAndError findClosest(double targetX, double targetZ) const;

private:
    /*
        DESCRIPTION: Precomputes all possible combinations of joint angles within given ranges and step size, calculating their corresponding end-effector positions using forward kinematics and storing them for later lookup
        INPUT: 
            - step: The rate of incraese in degrees
        ERRORS: N/A
    */
    void precompute(int step);

    /*
        DESCRIPTION: Builds a spatial grid by first determining the bounding box of all precomputed end-effector positions, padding it slightly, calculating the grid dimensions, and then assigning each position to a corresponding grid cell to enable fast spatial queries
        INPUT: N/A
        ERRORS: N/A
    */
    void buildGrid();

    double cellSize_;
    double minX_, maxX_, minZ_, maxZ_;
    int gridWidth_, gridHeight_;
    std::vector<AngleSet> angleSets_;
    std::vector<std::vector<size_t>> grid_;
};

/*
    DESCRIPTION: Converts the three joint angles (θ1, θ2, θ3) into corresponding actuator potentiometer values using linear equations derived from calibration data for each actuator’s range
    INPUT: 
        - theta1: Angle between Segment 1 and the rotating base
        - theta2: Angle between Segment 1 and Segment 2
        - theta3: Angle between Segment 2 and Segment 3
    ERRORS: N/A
*/
std::vector<double> calculatePotentiometerPositions(double theta1, double theta2, double theta3);

#endif // KINEMATICS_H*/