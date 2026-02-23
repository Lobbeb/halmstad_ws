package uav_ugv;

import inet.environment.common.PhysicalEnvironment;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;
import inet.common.geometry.common.IGeographicCoordinateSystem;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.applications.contract.IApp;

@license(LGPL);

// UAV
module UAV extends AdhocHost
{
    parameters:
        string mobilityType = default("SuperpositioningMobility");
        @display("i=device/accesspoint;is=vs");
}

// UGV (Unmanned Ground Vehicle)
module UGV extends AdhocHost
{
    parameters:
        string mobilityType = default("GaussMarkovMobility");
        @display("i=device/accesspoint;is=vs");
}

network UAV2UGVSimulation
{
    parameters:
        bool enableGazeboBridge = default(false);
        @display("bgb=0,0");
    submodules:
        gazeboScheduler: GazeboPositionScheduler if enableGazeboBridge {
            @display("p=30,310");
        }
        visualizer: IntegratedVisualizer {
            @display("p=30,267");
        }
        coordinateSystem: <default("SimpleGeographicCoordinateSystem")> like IGeographicCoordinateSystem {
            @display("p=182,225");
        }

        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=30,224");
        }

        physicalEnvironment: PhysicalEnvironment {
            @display("p=182,267");
        }

        configurator: Ipv4NetworkConfigurator {
            @display("p=182,186");
        }

        // UGV (Unmanned Ground Vehicle)
        ugv: UGV {
            @display("p=20,20");
        }

        // UAV (Unmanned Aerial Vehicle)
        uav: UAV {
            @display("p=60,60");
        }
}
