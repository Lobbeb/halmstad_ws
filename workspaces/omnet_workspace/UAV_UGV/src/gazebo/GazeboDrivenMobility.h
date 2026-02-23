// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>

#include <omnetpp.h>

#include "inet/mobility/single/GaussMarkovMobility.h"

class GazeboDrivenMobility : public inet::GaussMarkovMobility, public omnetpp::cListener
{
  protected:
    std::string schedulerModulePath;
    std::string trackedModel;
    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    double scaleX = 1.0;
    double scaleY = 1.0;
    double scaleZ = 1.0;
    double yawOffset = 0.0;          // radians
    bool alignFirstPoseToInitial = false;
    bool firstPoseAligned = false;
    bool ignoreZ = false;
    bool freezeAutonomousMotion = true;

    omnetpp::cModule *schedulerModule = nullptr;
    omnetpp::simsignal_t poseUpdatedSignal = omnetpp::cComponent::registerSignal("gazeboPoseUpdated");

  protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void move() override;
    virtual void receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signal, omnetpp::cObject *object, omnetpp::cObject *details) override;

  public:
    virtual const inet::Coord& getCurrentPosition() override;
    virtual const inet::Coord& getCurrentVelocity() override;
};
