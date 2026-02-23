package uav_ugv;

simple GazeboPositionScheduler
{
    parameters:
        string host = default("127.0.0.1");
        int port = default(5555);
        double updateInterval @unit(s) = default(0.1s);
        double connectAt @unit(s) = default(0s);
        int recvTimeoutMs = default(100);
        @display("i=block/wrench");
}
