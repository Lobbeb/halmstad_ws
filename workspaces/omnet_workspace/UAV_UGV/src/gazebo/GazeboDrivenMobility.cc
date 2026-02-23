// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GazeboDrivenMobility.h"

#include "GazeboPositionScheduler.h"
#include "inet/common/InitStages.h"
#include <cmath>

Define_Module(GazeboDrivenMobility);

namespace {
double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
}

void GazeboDrivenMobility::initialize(int stage)
{
    GaussMarkovMobility::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        schedulerModulePath = par("schedulerModule").stdstringValue();
        trackedModel = par("trackedModel").stdstringValue();
        offsetX = par("offsetX").doubleValue();
        offsetY = par("offsetY").doubleValue();
        offsetZ = par("offsetZ").doubleValue();
        scaleX = par("scaleX").doubleValue();
        scaleY = par("scaleY").doubleValue();
        scaleZ = par("scaleZ").doubleValue();
        yawOffset = par("yawOffset").doubleValueInUnit("rad");
        alignFirstPoseToInitial = par("alignFirstPoseToInitial").boolValue();
        firstPoseAligned = false;
        ignoreZ = par("ignoreZ").boolValue();
        freezeAutonomousMotion = par("freezeAutonomousMotion").boolValue();

        // This mobility is driven by external Gazebo updates.
        if (freezeAutonomousMotion) {
            stationary = true;
            nextChange = -1;
            lastVelocity = inet::Coord::ZERO;
            lastAngularVelocity = inet::Quaternion::IDENTITY;
        }
    }
    else if (stage == inet::INITSTAGE_LAST) {
        schedulerModule = getModuleByPath(schedulerModulePath.c_str());
        if (schedulerModule == nullptr) {
            throw omnetpp::cRuntimeError("GazeboDrivenMobility could not find scheduler module: %s", schedulerModulePath.c_str());
        }
        schedulerModule->subscribe(poseUpdatedSignal, this);
    }
}

void GazeboDrivenMobility::finish()
{
    if (schedulerModule != nullptr) {
        schedulerModule->unsubscribe(poseUpdatedSignal, this);
        schedulerModule = nullptr;
    }
}

void GazeboDrivenMobility::move()
{
    if (!freezeAutonomousMotion)
        GaussMarkovMobility::move();
}

const inet::Coord& GazeboDrivenMobility::getCurrentPosition()
{
    if (!freezeAutonomousMotion)
        return GaussMarkovMobility::getCurrentPosition();
    return lastPosition;
}

const inet::Coord& GazeboDrivenMobility::getCurrentVelocity()
{
    if (!freezeAutonomousMotion)
        return GaussMarkovMobility::getCurrentVelocity();
    return lastVelocity;
}

void GazeboDrivenMobility::receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signal, omnetpp::cObject *object, omnetpp::cObject *details)
{
    if (signal != poseUpdatedSignal)
        return;

    auto *pose = dynamic_cast<GazeboModelPose *>(object);
    if (pose == nullptr)
        return;

    if (!trackedModel.empty() && pose->modelName != trackedModel)
        return;

    Enter_Method("%s", omnetpp::cComponent::getSignalName(signal));

    const double mappedX = pose->x * scaleX;
    const double mappedY = pose->y * scaleY;
    const double mappedZ = pose->z * scaleZ;

    if (alignFirstPoseToInitial && !firstPoseAligned) {
        // Keep the first bridged sample at the configured initial OMNeT position.
        offsetX = lastPosition.x - mappedX;
        offsetY = lastPosition.y - mappedY;
        if (!ignoreZ)
            offsetZ = lastPosition.z - mappedZ;
        firstPoseAligned = true;
    }

    inet::Coord newPosition(
        mappedX + offsetX,
        mappedY + offsetY,
        mappedZ + offsetZ
    );
    if (ignoreZ)
        newPosition.z = lastPosition.z;

    const bool positionChanged = newPosition != lastPosition;

    auto now = omnetpp::simTime();
    if (positionChanged && now > lastUpdate)
        lastVelocity = (newPosition - lastPosition) / (now - lastUpdate).dbl();
    else
        lastVelocity = inet::Coord::ZERO;

    inet::Quaternion newOrientation = lastOrientation;
    if (pose->hasYaw) {
        // Respect axis reflections/scales when mapping heading between Gazebo and OMNeT frames.
        const double transformedYaw = std::atan2(scaleY * std::sin(pose->yaw), scaleX * std::cos(pose->yaw));
        const double headingYaw = normalizeAngle(transformedYaw + yawOffset);
        newOrientation = inet::Quaternion(inet::EulerAngles(inet::rad(headingYaw), inet::rad(0.0), inet::rad(0.0)));
    }
    else if (lastVelocity != inet::Coord::ZERO) {
        inet::Coord direction = lastVelocity;
        direction.normalize();
        auto alpha = inet::rad(std::atan2(direction.y, direction.x));
        auto beta = inet::rad(-std::asin(direction.z));
        auto gamma = inet::rad(0.0);
        newOrientation = inet::Quaternion(inet::EulerAngles(alpha, beta, gamma));
    }

    const bool orientationChanged = newOrientation != lastOrientation;
    if (!positionChanged && !orientationChanged)
        return;

    lastPosition = newPosition;
    targetPosition = newPosition;
    lastOrientation = newOrientation;
    nextChange = -1;
    lastUpdate = now;

    if (par("updateDisplayString").boolValue())
        updateDisplayStringFromMobilityState();

    emitMobilityStateChangedSignal();
}
