/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ns3/applications-module.h"
#include "ns3/bridge-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include "flow-metrics.h"

#include <algorithm>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

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

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("FirstScriptExample");

static vector<string>
ParseRateSweep(const string& raw)
{
    vector<string> values;
    stringstream ss(raw);
    string item;
    while (getline(ss, item, ','))
    {
        if (!item.empty())
        {
            auto begin = item.find_first_not_of(" \t\r\n");
            auto end = item.find_last_not_of(" \t\r\n");
            if (begin == string::npos)
            {
                continue;
            }
            values.push_back(item.substr(begin, end - begin + 1));
        }
    }
    return values;
}

static void
RunSimulationAtRate(const string& videoRate)
{
    cout << "\n=== Video traffic rate: " << videoRate << " ===" << endl;

    map<uint32_t, uint64_t> lastRxBytes;
    NodeContainer sourceNode;
    sourceNode.Create(1);

    NodeContainer ecuNodes;
    ecuNodes.Create(4);

    NodeContainer csmaSwitch;
    csmaSwitch.Create(1);

    NodeContainer terminals;
    terminals.Add(sourceNode.Get(0));
    terminals.Add(ecuNodes);

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1Gbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    NetDeviceContainer terminalDevices;
    NetDeviceContainer switchPorts;
    for (uint32_t i = 0; i < terminals.GetN(); ++i)
    {
        NetDeviceContainer link = csma.Install(NodeContainer(terminals.Get(i), csmaSwitch.Get(0)));
        terminalDevices.Add(link.Get(0));
        switchPorts.Add(link.Get(1));
    }

    // Model the switching element as a bridge and enforce TSN-style strict-priority
    // scheduling on each switch port. The three ECUs are connected to the switch on separate
    // CSMA links, and the traffic classes are mapped as:
    // ECU traffic -> priority 0 (higher priority)
    // video traffic -> priority 1 (lower priority)
    Ptr<Node> switchNode = csmaSwitch.Get(0);
    BridgeHelper bridge;
    bridge.Install(switchNode, switchPorts);

    Ptr<TrafficControlLayer> trafficControl = terminals.Get(0)->GetObject<TrafficControlLayer>();
    Ptr<QueueDisc> rootQueue;
    if (trafficControl)
    {
        TrafficControlHelper tsnTch;
        uint16_t tsnQueueHandle = tsnTch.SetRootQueueDisc("ns3::PrioQueueDisc",
                                                         "Priomap",
                                                         StringValue("0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1"));
        TrafficControlHelper::ClassIdList tsnClasses =
            tsnTch.AddQueueDiscClasses(tsnQueueHandle, 2, "ns3::QueueDiscClass");
        tsnTch.AddChildQueueDisc(tsnQueueHandle, tsnClasses[0], "ns3::FifoQueueDisc");
        tsnTch.AddChildQueueDisc(tsnQueueHandle, tsnClasses[1], "ns3::FifoQueueDisc");

        QueueDiscContainer qdiscs = tsnTch.Install(terminalDevices);
        rootQueue = qdiscs.Get(0);
    }

    InternetStackHelper stack;
    stack.SetIpv6StackInstall(false);
    stack.Install(terminals);
    stack.Install(csmaSwitch);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer terminalInterfaces = address.Assign(terminalDevices);

    const uint16_t videoPort = 9;
    const uint16_t ecu1Port = 10;
    const uint16_t ecu2Port = 11;
    const uint16_t ecu3Port = 12;

    // Video traffic: priority 1 (lower priority class) from source to ECU 1.
    OnOffHelper videoSource("ns3::UdpSocketFactory",
                            InetSocketAddress(terminalInterfaces.GetAddress(1), videoPort));
    videoSource.SetAttribute("OnTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    videoSource.SetAttribute("OffTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    videoSource.SetConstantRate(DataRate(videoRate));
    videoSource.SetAttribute("PacketSize", UintegerValue(1400));

    ApplicationContainer sourceApps = videoSource.Install(sourceNode.Get(0));
    sourceApps.Start(Seconds(1));
    sourceApps.Stop(Seconds(10));
    std::cout << "Traffic class assignment: ECU telemetry => priority 0, video stream => priority 1"
              << std::endl;

    PacketSinkHelper videoSink("ns3::UdpSocketFactory",
                               Address(InetSocketAddress(Ipv4Address::GetAny(), videoPort)));
    ApplicationContainer videoSinkApps = videoSink.Install(ecuNodes.Get(0));
    videoSinkApps.Start(Seconds(1));
    videoSinkApps.Stop(Seconds(10));

    // ECU 2 telemetry: priority 0 (higher priority class)
    OnOffHelper ecu1Control("ns3::UdpSocketFactory",
                            InetSocketAddress(terminalInterfaces.GetAddress(0), ecu1Port));
    ecu1Control.SetAttribute("OnTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    ecu1Control.SetAttribute("OffTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ecu1Control.SetConstantRate(DataRate("32kbps"));
    ecu1Control.SetAttribute("PacketSize", UintegerValue(256));

    ApplicationContainer ecu1Apps = ecu1Control.Install(ecuNodes.Get(1));
    ecu1Apps.Start(Seconds(1));
    ecu1Apps.Stop(Seconds(10));

    PacketSinkHelper ecu1Sink("ns3::UdpSocketFactory",
                             Address(InetSocketAddress(Ipv4Address::GetAny(), ecu1Port)));
    ApplicationContainer ecu1SinkApps = ecu1Sink.Install(sourceNode.Get(0));
    ecu1SinkApps.Start(Seconds(1));
    ecu1SinkApps.Stop(Seconds(10));

    // ECU 3 telemetry: priority 0 (higher priority class)
    OnOffHelper ecu2Control("ns3::UdpSocketFactory",
                            InetSocketAddress(terminalInterfaces.GetAddress(0), ecu2Port));
    ecu2Control.SetAttribute("OnTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    ecu2Control.SetAttribute("OffTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ecu2Control.SetConstantRate(DataRate("24kbps"));
    ecu2Control.SetAttribute("PacketSize", UintegerValue(256));

    ApplicationContainer ecu2Apps = ecu2Control.Install(ecuNodes.Get(2));
    ecu2Apps.Start(Seconds(1));
    ecu2Apps.Stop(Seconds(10));

    PacketSinkHelper ecu2Sink("ns3::UdpSocketFactory",
                             Address(InetSocketAddress(Ipv4Address::GetAny(), ecu2Port)));
    ApplicationContainer ecu2SinkApps = ecu2Sink.Install(sourceNode.Get(0));
    ecu2SinkApps.Start(Seconds(1));
    ecu2SinkApps.Stop(Seconds(10));

    // ECU 3 telemetry: priority 0 (higher priority class)
    OnOffHelper ecu3Control("ns3::UdpSocketFactory",
                            InetSocketAddress(terminalInterfaces.GetAddress(0), ecu3Port));
    ecu3Control.SetAttribute("OnTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    ecu3Control.SetAttribute("OffTime",
                             StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ecu3Control.SetConstantRate(DataRate("24kbps"));
    ecu3Control.SetAttribute("PacketSize", UintegerValue(256));

    ApplicationContainer ecu3Apps = ecu3Control.Install(ecuNodes.Get(3));
    ecu3Apps.Start(Seconds(1));
    ecu3Apps.Stop(Seconds(10));

    PacketSinkHelper ecu3Sink("ns3::UdpSocketFactory",
                             Address(InetSocketAddress(Ipv4Address::GetAny(), ecu3Port)));
    ApplicationContainer ecu3SinkApps = ecu3Sink.Install(sourceNode.Get(0));
    ecu3SinkApps.Start(Seconds(1));
    ecu3SinkApps.Stop(Seconds(10));


    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::cout << "Priority queue policy: band 0 (ECU control) is higher priority than band 1 "
              << "(video stream). FlowMonitor delay and loss metrics below show the queueing "
              << "behavior under saturation."
              << std::endl;

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();
    if (rootQueue)
    {
        Simulator::Schedule(Seconds(1), &ns3::flowmetrics::PrintQueueStats, rootQueue);
    }
    Simulator::Schedule(Seconds(1),
                       &ns3::flowmetrics::PrintThroughputOnce,
                       monitor,
                       ref(lastRxBytes));
    Simulator::Stop(Seconds(10.5));

    Simulator::Run();
    ns3::flowmetrics::PrintFlowSummary(monitor);
    Simulator::Destroy();
}

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    string videoRate = "100Mbps";
    string videoRateSweep;

    cmd.AddValue("video-rate", "Single video source rate to test", videoRate);
    cmd.AddValue("video-rate-sweep",
                 "Comma-separated list of video traffic rates to sweep, e.g. 50Mbps,100Mbps,250Mbps,500Mbps",
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