#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>

using namespace std;
struct Vehicle {
    string name;
    double mass;
    double cd;
    double area;
    double radius;
    double tireMu;
    double peakTorque;
    double redlineRPM;
};

Vehicle readVehicle(const string& fileName) {
    ifstream file("input/" + fileName);
    Vehicle car{};
    if (!file.is_open()) {
        cout << "Error: Could not open file " << fileName << endl;
        return car;
    }
    if (!getline(file, car.name) || !(file >> car.mass >> car.cd >> car.area >> car.radius >> car.tireMu >> car.peakTorque >> car.redlineRPM)) {
        cout << "Error: Invalid vehicle data in " << fileName << endl;
        return car;
    }
    if (car.mass <= 0.0 || car.cd < 0.0 || car.area <= 0.0 || car.radius <= 0.0 || car.tireMu <= 0.0 || car.peakTorque <= 0.0 || car.redlineRPM <= 0.0) {
        cout << "Error: Invalid vehicle data in " << fileName << endl;
        return car;
    }
    file.close();
    cout << "Loaded vehicle: " << car.name << endl;
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
vector<Data> runSim(const Vehicle& car) {
    vector<Data> data;
    
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
            cout << "Gear shifted at " << fixed << setprecision(2) << time << " seconds" << " from " << currentGear << " to " << currentGear + 1 << " at " << fixed << setprecision(2) << (carVel * meterToMph) << " mph" << endl;
            currentGear++;
            currentTotalRatio = gearRatios[currentGear - 1] * finalDrive;
            engineRPM = wheelAngularVel * currentTotalRatio * (60.0 / (2.0 * M_PI));
            engineRPM = max(engineRPM, idleRPM);
            shiftTimer = shiftDuration;
        }

        double F_drag = 0.5 * airDensity * car.cd * pow(carVel, 2) * car.area;
        double F_roll = Crr * car.mass * g;
        double F_traction = 0.0;

        bool currentlyShifting = (shiftTimer > 0.0);

        if (currentlyShifting) {
            F_traction = 0.0;
            shiftTimer = max(0.0, shiftTimer - timeStep);
        }
        else {
            double maxGrip = car.tireMu * car.mass * g;
            double rawTraction = (getEngineTorque(engineRPM, car.peakTorque, car.redlineRPM) * currentTotalRatio * drivetrainEfficiency) / car.radius;
            F_traction = min(rawTraction, maxGrip);
        }

        double F_net = F_traction - F_drag - F_roll;
        acceleration = F_net / car.mass;
        data.push_back({ time, carVel * meterToMph, acceleration / g, currentGear, engineRPM, distance });
        double oldVelocity = carVel;
        double oldDistance = distance;
        double oldTime = time;
        distance += carVel * timeStep + 0.5 * acceleration * timeStep * timeStep;
        carVel += acceleration * timeStep;
        carVel = max(0.0, carVel);
        time += timeStep;
        if (!sixtyRecorded && oldVelocity < sixtySpeed && carVel >= sixtySpeed) {
            double fraction = (sixtySpeed - oldVelocity) / (carVel - oldVelocity);
            double zeroToSixtyTime = oldTime + fraction * timeStep;
            cout << "0-60 mph in " << fixed << setprecision(3) << zeroToSixtyTime << " seconds" << endl;
            sixtyRecorded = true;
        }
        if (!quarterRecorded && oldDistance < quarterDistance && distance >= quarterDistance) {
            double fraction = (quarterDistance - oldDistance) /(distance - oldDistance);
            double quarterMileTime = oldTime + fraction * timeStep;
            cout << "Quarter mile time: " << fixed << setprecision(3) << quarterMileTime << " seconds" << endl;
            quarterRecorded = true;
        }
        if (currentGear == numGears && engineRPM >= car.redlineRPM) {
            cout << "Top speed (gear limited) reached at: " << fixed << setprecision(3) << (carVel * meterToMph) << " mph at " << time << " seconds" << endl;
            topSpeed = true;
            break;
        }
        if (currentGear == numGears && acceleration < 0.05  && !currentlyShifting) {
            cout << "Top speed reached at: " << (carVel * meterToMph) << " mph at " << fixed << setprecision(3) << time << " seconds" << endl;
            topSpeed = true;
            break;
        }
    }
    
    if (!quarterRecorded) {
        cout << "Your car didn't complete a quarter mile in " << maxTime << " seconds" << endl;
    }
    if (!topSpeed) {
        cout << "Your car didn't reach it's top speed in " << maxTime << " seconds" << endl;
    }
    return data;
}
void CSV(const string& fileName, const vector<Data>& data) {
    ofstream csv(fileName);
    if (!csv.is_open()) {
        cout << "Error: Could not create CSV file " << fileName << endl;
        return;
    }
    csv << "Time_s,Speed_mph,Acceleration_G,Gear,Engine_RPM,Distance_m" << endl;
    for (const Data& dp : data) {
        csv << fixed << setprecision(3) << dp.time << "," << dp.speed << "," << dp.accel << "," << dp.gear << "," << dp.engineRPM << "," << dp.distanceMeters << endl;
    }
    cout << "Data exported to " << fileName << endl;
}

int main() {
    vector<Vehicle> cars = {readVehicle("chevy.txt"), readVehicle("corolla.txt"), readVehicle("hellcat.txt"), readVehicle("honda_civic.txt"), readVehicle("m3.txt"), readVehicle("mustang.txt"), readVehicle("porsche_911_gt3.txt"), readVehicle("silverado.txt")};
    for (const Vehicle& car : cars) {
        cout << car.name << endl;
        vector<Data> allCarData = runSim(car);
        CSV("results/" + car.name + "_telemetry.csv", allCarData);
        cout << " " << endl;
    }
    return 0;
}
