#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>


struct Vehicle {
    std::string name;
    double mass;
    double cd;
    double area;
    double radius;
    double tireMu;
    double peakTorque;
    double redlineRPM;
};

Vehicle readVehicle(const std::string& fileName) {
    std::ifstream file("input/" + fileName);
    Vehicle car{};
    if (!file.is_open()) {
        std::cout << "Error: Could not open file " << fileName << std::endl;
        return car;
    }
    if (!std::getline(file, car.name) || !(file >> car.mass >> car.cd >> car.area >> car.radius >> car.tireMu >> car.peakTorque >> car.redlineRPM)) {
        std::cout << "Error: Invalid vehicle data in " << fileName << std:endl;
        return car;
    }
    if (car.mass <= 0.0 || car.cd < 0.0 || car.area <= 0.0 || car.radius <= 0.0 || car.tireMu <= 0.0 || car.peakTorque <= 0.0 || car.redlineRPM <= 0.0) {
        std::cout << "Error: Invalid vehicle data in " << fileName << std:endl;
        return car;
    }
    file.close();
    std::cout << "Loaded vehicle: " << car.name << std:endl;
    return car;
}


double getEngineTorque(double rpm, double peakTorque, double redlineRPM) {
    double rpmPct = rpm / redlineRPM;
    if (rpmPct < 0.15) {
        return peakTorque * 0.50;
    }
    if (rpmPct < 0.30) {
        return peakTorque * 0.75;
    }
    if (rpmPct < 0.45) {
        return peakTorque * 0.95;
    }
    if (rpmPct < 0.65) {
        return peakTorque * 1.00;
    }
    if (rpmPct < 0.85) {
        return peakTorque * 0.90;
    }
    if (rpmPct <= 1.0) {
        return peakTorque * 0.75;
    }
    return 0.0;
}
struct Data {
    double time;
    double speed;
    double accel;
    int gear;
    double engineRPM;
    double distanceMeters;
};
std::vector<Data> runSim(const Vehicle& car) {
    std::vector<Data> data;
    
    constexpr double airDensity = 1.225;
    constexpr double g = 9.81;
    constexpr double Crr = 0.015;
    constexpr double timeStep = 0.01;
    constexpr double sixtySpeed = 26.82;
    constexpr double quarterDistance = 402.3;
    const double gearRatios[] = { 3.50, 2.10, 1.40, 1.00, 0.65 };
    const int numGears = 5;
    const double finalDrive = 3.73;
    const double idleRPM = 1000.0;
    constexpr double meterToMph = 2.23694;
    constexpr double shiftDuration = 0.20;
    constexpr double drivetrainEfficiency = 0.90;
    constexpr double maxTime = 180.0;
    
    double carVel = 0.0;
    double distance = 0.0;
    double time = 0.0;
    double acceleration = 0.0;
    bool sixtyRecorded = false;
    bool quarterRecorded = false;
    bool topSpeed = false;
    int currentGear = 1;
    double shiftTimer = 0.0;
    
    while (time < maxTime) {
        double wheelAngularVel = carVel / car.radius;
        double currentTotalRatio = gearRatios[currentGear - 1] * finalDrive;
        double engineRPM = wheelAngularVel * currentTotalRatio * (60.0 / (2.0 * M_PI));
        if (engineRPM < idleRPM) {
            engineRPM = idleRPM;
        }
        if (engineRPM >= car.redlineRPM && currentGear < numGears && shiftTimer <= 0.0) {
            std::cout << "Gear shifted at " << std::fixed << std::setprecision(2) << time << " seconds" << " from " << currentGear << " to " << currentGear + 1 << " at " << fixed << setprecision(2) << (carVel * meterToMph) << " mph" << std:endl;
            currentGear++;
            currentTotalRatio = gearRatios[currentGear - 1] * finalDrive;
            engineRPM = wheelAngularVel * currentTotalRatio * (60.0 / (2.0 * M_PI));
            engineRPM = std::max(engineRPM, idleRPM);
            shiftTimer = shiftDuration;
        }

        double F_drag = 0.5 * airDensity * car.cd * std::pow(carVel, 2) * car.area;
        double F_roll = Crr * car.mass * g;
        double F_traction = 0.0;

        bool currentlyShifting = (shiftTimer > 0.0);

        if (currentlyShifting) {
            F_traction = 0.0;
            shiftTimer = std::max(0.0, shiftTimer - timeStep);
        }
        else {
            double maxGrip = car.tireMu * car.mass * g;
            double rawTraction = (getEngineTorque(engineRPM, car.peakTorque, car.redlineRPM) * currentTotalRatio * drivetrainEfficiency) / car.radius;
            F_traction = std::min(rawTraction, maxGrip);
        }

        double F_net = F_traction - F_drag - F_roll;
        acceleration = F_net / car.mass;
        data.push_back({ time, carVel * meterToMph, acceleration / g, currentGear, engineRPM, distance });
        double oldVelocity = carVel;
        double oldDistance = distance;
        double oldTime = time;
        distance += carVel * timeStep + 0.5 * acceleration * timeStep * timeStep;
        carVel += acceleration * timeStep;
        carVel = std::max(0.0, carVel);
        time += timeStep;
        if (!sixtyRecorded && oldVelocity < sixtySpeed && carVel >= sixtySpeed) {
            double fraction = (sixtySpeed - oldVelocity) / (carVel - oldVelocity);
            double zeroToSixtyTime = oldTime + fraction * timeStep;
            std::cout << "0-60 mph in " << std::fixed << std::setprecision(3) << zeroToSixtyTime << " seconds" << std:endl;
            sixtyRecorded = true;
        }
        if (!quarterRecorded && oldDistance < quarterDistance && distance >= quarterDistance) {
            double fraction = (quarterDistance - oldDistance) /(distance - oldDistance);
            double quarterMileTime = oldTime + fraction * timeStep;
            std::cout << "Quarter mile time: " << std::fixed << std::setprecision(3) << quarterMileTime << " seconds" << std:endl;
            quarterRecorded = true;
        }
        if (currentGear == numGears && engineRPM >= car.redlineRPM) {
            std::cout << "Top speed (gear limited) reached at: " << std::fixed << std::setprecision(3) << (carVel * meterToMph) << " mph at " << time << " seconds" << std:endl;
            topSpeed = true;
            break;
        }
        if (currentGear == numGears && acceleration < 0.05  && !currentlyShifting) {
            std::cout << "Top speed reached at: " << (carVel * meterToMph) << " mph at " << std::fixed << std::setprecision(3) << time << " seconds" << std:endl;
            topSpeed = true;
            break;
        }
    }
    
    if (!quarterRecorded) {
        std::cout << "Your car didn't complete a quarter mile in " << maxTime << " seconds" << std:endl;
    }
    if (!topSpeed) {
        std::cout << "Your car didn't reach it's top speed in " << maxTime << " seconds" << std:endl;
    }
    return data;
}
void CSV(const std::string& fileName, const std::vector<Data>& data) {
    std::ofstream csv(fileName);
    if (!csv.is_open()) {
        std::cout << "Error: Could not create CSV file " << fileName << std:endl;
        return;
    }
    csv << "Time_s,Speed_mph,Acceleration_G,Gear,Engine_RPM,Distance_m" << std:endl;
    for (const Data& dp : data) {
        csv << std::fixed << std::setprecision(3) << dp.time << "," << dp.speed << "," << dp.accel << "," << dp.gear << "," << dp.engineRPM << "," << dp.distanceMeters << std:endl;
    }
    std::cout << "Data exported to " << fileName << std:endl;
}

int main() {
    std::vector<Vehicle> cars = {readVehicle("chevy.txt"), readVehicle("corolla.txt"), readVehicle("hellcat.txt"), readVehicle("honda_civic.txt"), readVehicle("m3.txt"), readVehicle("mustang.txt"), readVehicle("porsche_911_gt3.txt"), readVehicle("silverado.txt")};
    for (const Vehicle& car : cars) {
        std::cout << car.name << std:endl;
        std::vector<Data> allCarData = runSim(car);
        CSV("results/" + car.name + "_telemetry.csv", allCarData);
        std::cout << " " << std:endl;
    }
    return 0;
}
