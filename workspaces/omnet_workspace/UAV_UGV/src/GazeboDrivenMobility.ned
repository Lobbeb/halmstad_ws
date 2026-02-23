package uav_ugv;

import inet.mobility.single.GaussMarkovMobility;

simple GazeboDrivenMobility extends GaussMarkovMobility
{
    parameters:
        string schedulerModule = default("^.^.gazeboScheduler");
        string trackedModel = default("x2_ugv");
        double offsetX @unit(m) = default(0m);
        double offsetY @unit(m) = default(0m);
        double offsetZ @unit(m) = default(0m);
        double scaleX = default(1);
        double scaleY = default(1);
        double scaleZ = default(1);
        double yawOffset @unit(deg) = default(0deg);
        bool alignFirstPoseToInitial = default(false);
        bool ignoreZ = default(false);
        bool freezeAutonomousMotion = default(true);
        @class(GazeboDrivenMobility);
}
