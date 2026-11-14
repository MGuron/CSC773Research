/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Layer-2 automotive Ethernet switch baseline for ns-3.
 *
 * This version uses BridgeNetDevice for transparent MAC learning/forwarding.
 * The five terminal devices share one IPv4 subnet; the switch itself does
 * not run IPv4 and does not perform IP routing.
 *
 * IMPORTANT QoS LIMITATION:
 * The stock ns-3 BridgeNetDevice forwards received frames directly with
 * NetDevice::SendFrom(). It does not pass bridged frames through the normal
 * TrafficControlLayer queue-disc path. Therefore, this file provides a
 * finite per-device FIFO transmit queue, but NOT two class-specific queues
 * or strict-priority scheduling at the bridge egress ports. The IPv4 ToS
 * markings below identify the intended traffic classes only; they do not
 * cause priority scheduling in this stock bridge configuration.
 *
 * To add real per-port strict-priority egress scheduling, the bridge
 * forwarding path must be extended (or a custom bridge NetDevice created)
 * so frames are enqueued into per-port high/low queues before SendFrom().
 */

#include "ns3/applications-module.h"
#include "ns3/bridge-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"

#include "flow-metrics.h"

#include <cstdint>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("AutomotiveEthernetLayer2Switch");

namespace
{
constexpr uint32_t kNumEcus = 4;
constexpr uint32_t kNumTerminals = 5; // one video source + four ECUs
constexpr uint32_t kDeviceTxQueuePackets = 100;

//TODO:
// Figure out the vehicle level timing requirements - 
// Import some actual CAN data, make sure the 1 PGN : Datagram relationship is kept
// Set up ECUs 2-4 to actually send this data instead of the normal data?
// Multiple different priorities for this ECU data
// Make a parser to read results? Matlab or Python that manually will make some graphs or something

// Default Network Topology
//
//       10.1.1.0
// n0 (video source) ----- n1 (switch) - n3 (ECUs with control data)
//                         |           - n4
//                         |           - n5
//                         |
//                       n2 (automotive ECU)
//                     LAN 10.1.2.0
//

// These ToS values identify the intended traffic classes. With the stock
// BridgeNetDevice in this file, they are NOT used for strict-priority output
// scheduling. A custom bridge egress scheduler is required for that behavior.
constexpr uint8_t kJ1939Tos = 0xE0;
constexpr uint8_t kCameraTos = 0x00;

vector<string>
ParseRateSweep(const string& raw)
{
    vector<string> values;
    stringstream ss(raw);
    string item;

    while (getline(ss, item, ','))
    {
        if (!item.empty())
        {
            const auto begin = item.find_first_not_of(" \t\r\n");
            const auto end = item.find_last_not_of(" \t\r\n");
            if (begin != string::npos)
            {
                values.push_back(item.substr(begin, end - begin + 1));
            }
        }
    }
    return values;
}

void
RunSimulationAtRate(const string& videoRate)
{
    cout << "\n=== Camera traffic rate: " << videoRate << " ===" << endl;

    map<uint32_t, uint64_t> lastRxBytes;

    NodeContainer videoSource;
    videoSource.Create(1);

    NodeContainer ecuNodes;
    ecuNodes.Create(kNumEcus);

    NodeContainer switchNode;
    switchNode.Create(1);

    NodeContainer terminals;
    terminals.Add(videoSource.Get(0));
    terminals.Add(ecuNodes);

    // Each switch port is a separate two-device, full-duplex 1 Gbps link.
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(DataRate("1Gbps")));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    // Finite device-level transmit queue on each CSMA NetDevice. Each
    // switch-side NetDevice represents one physical switch egress port.
    csma.SetQueue("ns3::DropTailQueue<Packet>",
                  "MaxSize",
                  QueueSizeValue(QueueSize(to_string(kDeviceTxQueuePackets) + "p")));

    NetDeviceContainer terminalDevices;
    NetDeviceContainer switchPorts;

    for (uint32_t i = 0; i < terminals.GetN(); ++i)
    {
        NetDeviceContainer link =
            csma.Install(NodeContainer(terminals.Get(i), switchNode.Get(0)));

        terminalDevices.Add(link.Get(0));
        switchPorts.Add(link.Get(1));
    }

    // Install the Internet stack only on end systems. The switch is a
    // transparent Layer-2 bridge and has no IPv4 interfaces or routing table.
    InternetStackHelper stack;
    stack.SetIpv6StackInstall(false);
    stack.Install(terminals);

    // Create a learning bridge over all five switch-side Ethernet ports.
    BridgeHelper bridge;
    NetDeviceContainer bridgeDevice = bridge.Install(switchNode.Get(0), switchPorts);

    // All end systems are in one IP subnet because the switch bridges at L2.
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer terminalInterfaces = address.Assign(terminalDevices);

    // Destination is ECU 1. Camera and the three ECU telemetry streams
    // converge on the same switch egress port (the ECU 1-facing port).
    const Ipv4Address ecu1Address = terminalInterfaces.GetAddress(1);

    const uint16_t videoPort = 9;
    const uint16_t ecu1Port = 10;
    const uint16_t ecu2Port = 11;
    const uint16_t ecu3Port = 12;

    // Camera traffic: lower-priority class marking (ToS 0x00).
    OnOffHelper videoApp("ns3::UdpSocketFactory",
                         InetSocketAddress(ecu1Address, videoPort));
    videoApp.SetAttribute("OnTime",
                          StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    videoApp.SetAttribute("OffTime",
                          StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    videoApp.SetConstantRate(DataRate(videoRate));
    videoApp.SetAttribute("PacketSize", UintegerValue(1400));
    videoApp.SetAttribute("Tos", UintegerValue(kCameraTos));

    ApplicationContainer videoApps = videoApp.Install(videoSource.Get(0));
    videoApps.Start(Seconds(1));
    videoApps.Stop(Seconds(10));

    PacketSinkHelper videoSink(
        "ns3::UdpSocketFactory",
        Address(InetSocketAddress(Ipv4Address::GetAny(), videoPort)));
    ApplicationContainer videoSinkApps = videoSink.Install(ecuNodes.Get(0));
    videoSinkApps.Start(Seconds(1));
    videoSinkApps.Stop(Seconds(10));

    // J1939-related workload approximation: three UDP flows to ECU 1.
    // These are not actual J1939/CAN frames or a CAN-to-Ethernet gateway.
    const uint16_t controlPorts[] = {ecu1Port, ecu2Port, ecu3Port};
    const char* controlRates[] = {"32kbps", "24kbps", "24kbps"};

    for (uint32_t i = 0; i < 3; ++i)
    {
        OnOffHelper controlApp("ns3::UdpSocketFactory",
                               InetSocketAddress(ecu1Address, controlPorts[i]));
        controlApp.SetAttribute("OnTime",
                                StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        controlApp.SetAttribute("OffTime",
                                StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        controlApp.SetConstantRate(DataRate(controlRates[i]));
        controlApp.SetAttribute("PacketSize", UintegerValue(256));
        controlApp.SetAttribute("Tos", UintegerValue(kJ1939Tos));

        ApplicationContainer controlApps = controlApp.Install(ecuNodes.Get(i + 1));
        controlApps.Start(Seconds(1));
        controlApps.Stop(Seconds(10));

        PacketSinkHelper controlSink(
            "ns3::UdpSocketFactory",
            Address(InetSocketAddress(Ipv4Address::GetAny(), controlPorts[i])));
        ApplicationContainer controlSinkApps = controlSink.Install(ecuNodes.Get(0));
        controlSinkApps.Start(Seconds(1));
        controlSinkApps.Stop(Seconds(10));
    }

    cout << "Switch model: transparent Layer-2 BridgeNetDevice" << endl;
    cout << "Forwarding: MAC learning and Ethernet frame forwarding" << endl;
    cout << "Ports: " << kNumTerminals
         << " x 1 Gbps full-duplex point-to-point CSMA links" << endl;
    cout << "Device TX queue limit: " << kDeviceTxQueuePackets
         << " packets per NetDevice" << endl;
    cout << "Traffic markings: J1939-like ToS=0xE0; camera ToS=0x00" << endl;
    cout << "QoS limitation: stock BridgeNetDevice does not apply strict-priority" << endl;
    cout << "queue-disc scheduling to bridged egress frames." << endl;
    cout << "Camera and J1939-like flows converge on the ECU 1-facing bridge port." << endl;

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Schedule(Seconds(1),
                        &ns3::flowmetrics::PrintThroughputOnce,
                        monitor,
                        ref(lastRxBytes));

    Simulator::Stop(Seconds(10.5));
    Simulator::Run();
    ns3::flowmetrics::PrintFlowSummary(monitor);
    Simulator::Destroy();
}
} // namespace

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    string videoRate = "100Mbps";
    string videoRateSweep;

    cmd.AddValue("video-rate", "Single camera traffic rate to test", videoRate);
    cmd.AddValue("video-rate-sweep",
                 "Comma-separated camera traffic rates, e.g. 100Mbps,250Mbps,500Mbps,1Gbps",
                 videoRateSweep);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);
    LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    LogComponentEnable("PacketSink", LOG_LEVEL_INFO);

    vector<string> rates;
    if (videoRateSweep.empty())
    {
        rates.push_back(videoRate);
    }
    else
    {
        rates = ParseRateSweep(videoRateSweep);
        if (rates.empty())
        {
            rates.push_back(videoRate);
        }
    }

    for (const auto& rate : rates)
    {
        RunSimulationAtRate(rate);
    }

    return 0;
}
