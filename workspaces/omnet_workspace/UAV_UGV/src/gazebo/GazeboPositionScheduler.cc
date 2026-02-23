// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GazeboPositionScheduler.h"

#define WANT_WINSOCK2
#include <omnetpp/platdep/sockets.h>
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32) || defined(__CYGWIN__) || defined(_WIN64)
#include <ws2tcpip.h>
#else
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#endif

#include <cerrno>
#include <cstring>
#include <exception>
#include <sstream>

using omnetpp::cMessage;
using omnetpp::cRuntimeError;
using omnetpp::cSimpleModule;

static SOCKET socketFromPtr(void* ptr)
{
    return *static_cast<SOCKET*>(ptr);
}

Define_Module(GazeboPositionScheduler);

const omnetpp::simsignal_t GazeboPositionScheduler::poseUpdatedSignal =
    omnetpp::cComponent::registerSignal("gazeboPoseUpdated");

GazeboModelPose::GazeboModelPose(std::string modelName, double x, double y, double z, double yaw, bool hasYaw)
    : modelName(std::move(modelName))
    , x(x)
    , y(y)
    , z(z)
    , yaw(yaw)
    , hasYaw(hasYaw)
{
}

GazeboModelPose* GazeboModelPose::dup() const
{
    return new GazeboModelPose(*this);
}

GazeboPositionScheduler::GazeboPositionScheduler() = default;

GazeboPositionScheduler::~GazeboPositionScheduler()
{
    if (connectTimer) {
        cancelAndDelete(connectTimer);
        connectTimer = nullptr;
    }
    if (pollTimer) {
        cancelAndDelete(pollTimer);
        pollTimer = nullptr;
    }
    closeSocket();
}

const std::unordered_map<std::string, inet::Coord>& GazeboPositionScheduler::getLatestPoses() const
{
    return latestPoses;
}

void GazeboPositionScheduler::initialize()
{
    host = par("host").stdstringValue();
    port = par("port");
    updateInterval = par("updateInterval");
    connectAt = par("connectAt");
    recvTimeoutMs = par("recvTimeoutMs");

    connectTimer = new cMessage("gazeboConnect");
    pollTimer = new cMessage("gazeboPoll");

    if (connectAt <= SIMTIME_ZERO) {
        connectToBridge();
        scheduleAt(omnetpp::simTime() + updateInterval, pollTimer);
    }
    else {
        scheduleAt(connectAt, connectTimer);
    }
}

void GazeboPositionScheduler::handleMessage(cMessage* msg)
{
    if (msg == connectTimer) {
        connectToBridge();
        scheduleAt(omnetpp::simTime() + updateInterval, pollTimer);
        return;
    }

    if (msg == pollTimer) {
        if (connected) {
            pollOnce();
        }
        scheduleAt(omnetpp::simTime() + updateInterval, pollTimer);
        return;
    }

    delete msg;
}

void GazeboPositionScheduler::finish()
{
    closeSocket();
}

void GazeboPositionScheduler::connectToBridge()
{
    if (connected) {
        return;
    }

    if (initsocketlibonce() != 0) {
        throw cRuntimeError("Could not init socketlib");
    }

    in_addr addr;
    struct hostent* host_ent;
    struct in_addr saddr;

    saddr.s_addr = inet_addr(host.c_str());
    if (saddr.s_addr != static_cast<unsigned int>(-1)) {
        addr = saddr;
    }
    else if ((host_ent = gethostbyname(host.c_str()))) {
        addr = *((struct in_addr*) host_ent->h_addr_list[0]);
    }
    else {
        throw cRuntimeError("Invalid bridge address: %s", host.c_str());
    }

    sockaddr_in address;
    sockaddr* address_p = reinterpret_cast<sockaddr*>(&address);
    memset(address_p, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = addr.s_addr;

    SOCKET* sockPtr = new SOCKET();
    *sockPtr = ::socket(AF_INET, SOCK_STREAM, 0);
    if (*sockPtr == INVALID_SOCKET) {
        delete sockPtr;
        throw cRuntimeError("Could not create socket to connect to Gazebo bridge");
    }

    if (::connect(*sockPtr, address_p, sizeof(address)) < 0) {
        closesocket(*sockPtr);
        delete sockPtr;
        throw cRuntimeError("Could not connect to Gazebo bridge: %d: %s", sock_errno(), strerror(sock_errno()));
    }

    int noDelay = 1;
    ::setsockopt(*sockPtr, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&noDelay), sizeof(noDelay));

#if !defined(_WIN32) && !defined(__WIN32__) && !defined(WIN32) && !defined(__CYGWIN__) && !defined(_WIN64)
    if (recvTimeoutMs > 0) {
        timeval tv;
        tv.tv_sec = recvTimeoutMs / 1000;
        tv.tv_usec = (recvTimeoutMs % 1000) * 1000;
        ::setsockopt(*sockPtr, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
#endif

    socketPtr = sockPtr;
    connected = true;

    EV_INFO << "Connected to Gazebo pose bridge at " << host << ":" << port << omnetpp::endl;
}

void GazeboPositionScheduler::closeSocket()
{
    if (socketPtr) {
        closesocket(socketFromPtr(socketPtr));
        delete static_cast<SOCKET*>(socketPtr);
        socketPtr = nullptr;
    }
    connected = false;
}

void GazeboPositionScheduler::pollOnce()
{
    if (!sendLine("GET\n")) {
        EV_WARN << "Failed to send request to Gazebo bridge." << omnetpp::endl;
        return;
    }

    std::string line;
    if (!receiveLine(line)) {
        EV_WARN << "Failed to read response from Gazebo bridge." << omnetpp::endl;
        return;
    }

    std::vector<GazeboModelPose> poses;
    if (!parsePoseSnapshot(line, poses)) {
        EV_WARN << "Invalid pose snapshot format from Gazebo bridge: " << line << omnetpp::endl;
        return;
    }

    for (const auto& pose : poses) {
        latestPoses[pose.modelName] = inet::Coord(pose.x, pose.y, pose.z);

        GazeboModelPose signalPose(pose.modelName, pose.x, pose.y, pose.z, pose.yaw, pose.hasYaw);
        emit(poseUpdatedSignal, &signalPose);
    }
}

bool GazeboPositionScheduler::sendLine(const std::string& line)
{
    if (!socketPtr) {
        return false;
    }

    size_t totalSent = 0;
    while (totalSent < line.size()) {
        ssize_t sentBytes = ::send(socketFromPtr(socketPtr), line.c_str() + totalSent, line.size() - totalSent, 0);
        if (sentBytes > 0) {
            totalSent += static_cast<size_t>(sentBytes);
            continue;
        }
        if (sock_errno() == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

bool GazeboPositionScheduler::receiveLine(std::string& lineOut)
{
    if (!socketPtr) {
        return false;
    }

    lineOut.clear();
    char ch = 0;
    while (true) {
        int receivedBytes = ::recv(socketFromPtr(socketPtr), &ch, 1, 0);
        if (receivedBytes > 0) {
            if (ch == '\n') {
                break;
            }
            lineOut.push_back(ch);
            continue;
        }
        if (receivedBytes == 0) {
            closeSocket();
            return false;
        }
        if (sock_errno() == EINTR) {
            continue;
        }
        return false;
    }

    return true;
}

bool GazeboPositionScheduler::parsePoseSnapshot(const std::string& line, std::vector<GazeboModelPose>& poses) const
{
    // Expected format:
    // "<count> <name> <x> <y> <z> [<name> <x> <y> <z> ...]"
    // or
    // "<count> <name> <x> <y> <z> <yaw> [<name> <x> <y> <z> <yaw> ...]"
    std::istringstream iss(line);
    size_t count = 0;
    if (!(iss >> count)) {
        return false;
    }

    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }

    const size_t fieldsWithoutYaw = 4;
    const size_t fieldsWithYaw = 5;
    bool snapshotHasYaw = false;
    if (tokens.size() == count * fieldsWithoutYaw) {
        snapshotHasYaw = false;
    }
    else if (tokens.size() == count * fieldsWithYaw) {
        snapshotHasYaw = true;
    }
    else {
        return false;
    }

    poses.clear();
    poses.reserve(count);

    size_t tokenIndex = 0;
    for (size_t i = 0; i < count; ++i) {
        if (tokenIndex + fieldsWithoutYaw > tokens.size()) {
            return false;
        }

        const std::string& name = tokens[tokenIndex++];
        try {
            double x = std::stod(tokens[tokenIndex++]);
            double y = std::stod(tokens[tokenIndex++]);
            double z = std::stod(tokens[tokenIndex++]);
            if (snapshotHasYaw) {
                if (tokenIndex >= tokens.size()) {
                    return false;
                }
                double yaw = std::stod(tokens[tokenIndex++]);
                poses.emplace_back(name, x, y, z, yaw, true);
            }
            else {
                poses.emplace_back(name, x, y, z, 0.0, false);
            }
        }
        catch (const std::exception&) {
            return false;
        }
    }

    return true;
}
