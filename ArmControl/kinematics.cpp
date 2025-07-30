/*#include "kinematics.h"

void initTrigTables() {
    for (int deg = 0; deg < TABLE_SIZE; deg++) {
        double rad = deg * PI / 180.0;
        SIN_TABLE[deg] = std::sin(rad);
        COS_TABLE[deg] = std::cos(rad);
    }
}

inline int normalizeAngle(int deg) {
    deg %= TABLE_SIZE;
    return deg + (deg >> 31 & TABLE_SIZE);
}

Point forwardKinematics(double theta1, double theta2, double theta3) {
    int phi1_deg = static_cast<int>(theta1);
    int phi2_deg = static_cast<int>(theta1 + theta2 - 90.0);
    int phi3_deg = static_cast<int>(theta1 + theta2 + theta3 - 180.0);

    phi1_deg = normalizeAngle(phi1_deg);
    phi2_deg = normalizeAngle(phi2_deg);
    phi3_deg = normalizeAngle(phi3_deg);

    const double sin1 = SIN_TABLE[phi1_deg];
    const double cos1 = COS_TABLE[phi1_deg];
    const double sin2 = SIN_TABLE[phi2_deg];
    const double cos2 = COS_TABLE[phi2_deg];
    const double sin3 = SIN_TABLE[phi3_deg];
    const double cos3 = COS_TABLE[phi3_deg];

    double x1 = L * sin1;
    double z1 = L * cos1;
    double x2 = x1 + L * sin2;
    double z2 = z1 + L * cos2;
    double x3 = x2 + L * sin3;
    double z3 = z2 + L * cos3;

    return {x3, z3};
}


InverseKinematicsSolver::InverseKinematicsSolver(double cellSize, int step) : cellSize_(cellSize) {
    precompute(step);
    buildGrid();
}

AnglesAndError InverseKinematicsSolver::findClosest(double targetX, double targetZ) const {
    int gridX = static_cast<int>((targetX - minX_) / cellSize_);
    int gridZ = static_cast<int>((targetZ - minZ_) / cellSize_);

    double minDistSq = std::numeric_limits<double>::max();
    size_t bestIdx = 0;

    // Search in current cell and neighbors
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
            int nx = gridX + dx;
            int nz = gridZ + dz;
            if (nx >= 0 && nx < gridWidth_ && nz >= 0 && nz < gridHeight_) {
                size_t index = nz * gridWidth_ + nx;
                for (size_t idx : grid_[index]) {
                    const auto& set = angleSets_[idx];
                    double dx = set.pos.x - targetX;
                    double dz = set.pos.z - targetZ;
                    double distSq = dx * dx + dz * dz;
                    if (distSq < minDistSq) {
                        minDistSq = distSq;
                        bestIdx = idx;
                    }
                }
            }
        }
    }

    const auto& best = angleSets_[bestIdx];
    return {best.theta1, best.theta2, best.theta3, std::sqrt(minDistSq)};
}

void InverseKinematicsSolver::precompute(int step) {
    for (int theta1 = 75; theta1 <= 135; theta1 += step) {
        for (int theta2 = 70; theta2 <= 96; theta2 += step) {
            for (int theta3 = 90; theta3 <= 153; theta3 += step) {
                Point pos = forwardKinematics(theta1, theta2, theta3);
                angleSets_.push_back({static_cast<double>(theta1), static_cast<double>(theta2), static_cast<double>(theta3), pos});
            }
        }
    }
}

void InverseKinematicsSolver::buildGrid() {
    if (angleSets_.empty()) return;

    // Find limits
    minX_ = maxX_ = angleSets_[0].pos.x;
    minZ_ = maxZ_ = angleSets_[0].pos.z;
    for (const auto& set : angleSets_) {
        minX_ = std::min(minX_, set.pos.x);
        maxX_ = std::max(maxX_, set.pos.x);
        minZ_ = std::min(minZ_, set.pos.z);
        maxZ_ = std::max(maxZ_, set.pos.z);
    }

    // Adds padding
    minX_ -= cellSize_;
    maxX_ += cellSize_;
    minZ_ -= cellSize_;
    maxZ_ += cellSize_;

    //Calculates grid dimension
    gridWidth_ = static_cast<int>(std::ceil((maxX_ - minX_) / cellSize_)) + 1;
    gridHeight_ = static_cast<int>(std::ceil((maxZ_ - minZ_) / cellSize_)) + 1;
    grid_.resize(gridWidth_ * gridHeight_);

    // Populates grid
    for (size_t i = 0; i < angleSets_.size(); ++i) {
        const auto& set = angleSets_[i];
        int gridX = static_cast<int>((set.pos.x - minX_) / cellSize_);
        int gridZ = static_cast<int>((set.pos.z - minZ_) / cellSize_);
        size_t index = gridZ * gridWidth_ + gridX;
        grid_[index].push_back(i);
    }
}

std::vector<double> calculatePotentiometerPositions(double theta1, double theta2, double theta3) {
    // Actuator 1: Initial position 37 + 12.5 units / (theta1-75) degrees
    double p1 = 37.0 + 12.5 * (theta1 - 75.0);

    // Actuator 2: Initial position 952 + 33.9 units / (theta2 - 97) degrees
    double p2 = 952.0 + 33.9 * (theta2 - 97.0);

    // Actuator 3: Initial position 957 + 14.5 units / (theta3 - 154) degrees
    double p3 = 957.0 + 14.5 * (theta3 - 154.0);

    return {p1, p2, p3};
}

int main() {
    initTrigTables();

    InverseKinematicsSolver solver(2.0, 2);

    const double target_x = 135.0;
    const double target_z = 14.0;
    AnglesAndError result = solver.findClosest(target_x, target_z);

    std::cout << "Target position: (" << target_x << ", " << target_z << ") cm\n";
    std::cout << "Angles: theta1 = " << result.theta1 << "°, theta2 = " << result.theta2 
              << "°, theta3 = " << result.theta3 << "°\n";
    std::cout << "Error: " << result.error << " cm\n";
    
    return 0;
}*/