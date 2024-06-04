#ifndef VELOCITY_CURVE_GENERATOR_H
#define VELOCITY_CURVE_GENERATOR_H

#include <vector>
#include <cmath>  // For std::tanh

class VelocityCurveGenerator {
public:
    VelocityCurveGenerator(float startVelocity, float endVelocity, int numPoints = 100)
        : startVelocity(startVelocity), endVelocity(endVelocity), numPoints(numPoints) {
        generateCurve();
    }

    const std::vector<float>& getVelocities() const {
        //print velocities
        // for (int i = 0; i < velocities.size(); i++) {
        //     std::cout << velocities[i] << std::endl;
        // }
        return velocities;
    }

private:
    float startVelocity;
    float endVelocity;
    int numPoints;
    std::vector<float> velocities;

    void generateCurve() {
        velocities.clear();
        float range = endVelocity - startVelocity;
        float step = 1.0f / numPoints;

        for (float i = 0; i <= numPoints; ++i) {
            float t = i * step;
            float normalizedTime = t * 2.0f;
            float tanhValue = std::tanh(normalizedTime); // Tanh curve
            float velocity = startVelocity + (tanhValue) /  1.0f * range; // Scale and shift
            velocities.push_back(velocity);
        }
    } 
};

#endif // VELOCITY_CURVE_GENERATOR_H
