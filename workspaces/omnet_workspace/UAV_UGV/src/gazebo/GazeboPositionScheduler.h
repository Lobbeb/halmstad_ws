// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <omnetpp.h>

#include "inet/common/geometry/common/Coord.h"

class GazeboModelPose : public omnetpp::cObject {
public:
    GazeboModelPose() = default;
    GazeboModelPose(std::string modelName, double x, double y, double z, double yaw = 0.0, bool hasYaw = false);

    GazeboModelPose* dup() const override;

    std::string modelName;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    bool hasYaw = false;
};

class GazeboPositionScheduler : public omnetpp::cSimpleModule {
public:
    GazeboPositionScheduler();
    ~GazeboPositionScheduler() override;

    const std::unordered_map<std::string, inet::Coord>& getLatestPoses() const;

protected:
    void initialize() override;
    void handleMessage(omnetpp::cMessage* msg) override;
    void finish() override;

private:
    void connectToBridge();
    void closeSocket();
    void pollOnce();

    bool sendLine(const std::string& line);
    bool receiveLine(std::string& lineOut);
    bool parsePoseSnapshot(const std::string& line, std::vector<GazeboModelPose>& poses) const;

    std::string host;
    int port = 0;
    omnetpp::simtime_t updateInterval;
    omnetpp::simtime_t connectAt;
    int recvTimeoutMs = 100;

    void* socketPtr = nullptr;
    bool connected = false;

    omnetpp::cMessage* connectTimer = nullptr;
    omnetpp::cMessage* pollTimer = nullptr;

    std::unordered_map<std::string, inet::Coord> latestPoses;

    static const omnetpp::simsignal_t poseUpdatedSignal;
};
