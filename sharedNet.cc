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
#include "ns3/ipv4-flow-classifier.h"

#include "flow-metrics.h"
#include "asc-parser.h"

#include <cstdint>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>
#include <iomanip>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("AutomotiveEthernetLayer2Switch");

namespace{

    constexpr uint32_t kNumEcus = 4;
    constexpr uint32_t kNumTerminals = 5; // one video source + four ECUs
    constexpr uint32_t kDeviceTxQueuePackets = 100;

    //TODO:
    // Figure out the vehicle level timing requirements - 
    // Import some actual CAN data, make sure the 1 PGN : Datagram relationship is kept
    // Set up ECUs 2-4 to actually send this data instead of the normal data?
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

    void PrintJ1939FlowMetrics(Ptr<FlowMonitor> monitor,
                     FlowMonitorHelper& flowmonHelper)
    {   
        Ptr<Ipv4FlowClassifier> classifier =
            DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    
        if (!classifier)
        {
            NS_FATAL_ERROR("Unable to obtain IPv4 FlowMonitor classifier");
        }

        const auto stats = monitor->GetFlowStats();

        cout << "\n========== J1939 FLOW METRICS ==========" << endl;

        cout << left
            << setw(8)  << "Flow"
            << setw(18) << "Source IP"
            << setw(12) << "Src Port"
            << setw(12) << "Dst Port"
            << setw(12) << "Tx Pkts"
            << setw(12) << "Rx Pkts"
            << setw(12) << "Lost"
            << setw(14) << "Avg Delay"
            << setw(14) << "Avg Jitter"
            << setw(14) << "Throughput"
            << endl;

        for (const auto& [flowId, flowStats] : stats)
        {
            const auto tuple = classifier->FindFlow(flowId);

            // ASC-replay J1939 traffic uses UDP destination port 10.
            // If you use one port per priority, change this to:
            // tuple.destinationPort < 10 || tuple.destinationPort > 17
            if (tuple.protocol != 17 || tuple.destinationPort != 10)
            {
                continue;
            }

            const uint64_t lostPackets =
                flowStats.txPackets - flowStats.rxPackets;

            const double avgDelayMs =
                flowStats.rxPackets > 0
                    ? 1000.0 * flowStats.delaySum.GetSeconds()
                        / flowStats.rxPackets
                    : 0.0;

            const double avgJitterMs =
                flowStats.rxPackets > 1
                    ? 1000.0 * flowStats.jitterSum.GetSeconds()
                        / (flowStats.rxPackets - 1)
                    : 0.0;

            double throughputMbps = 0.0;

            if (flowStats.rxPackets > 0 &&
                flowStats.timeLastRxPacket > flowStats.timeFirstTxPacket)
            {
                const double duration =
                    (flowStats.timeLastRxPacket -
                    flowStats.timeFirstTxPacket).GetSeconds();

                throughputMbps =
                    (flowStats.rxBytes * 8.0) / duration / 1e6;
            }

            ostringstream sourceIp;
            sourceIp << tuple.sourceAddress;

            cout << left
                << setw(8)  << flowId
                << setw(18) << sourceIp.str()
                << setw(12) << tuple.sourcePort
                << setw(12) << tuple.destinationPort
                << setw(12) << flowStats.txPackets
                << setw(12) << flowStats.rxPackets
                << setw(12) << lostPackets
                << setw(14) << avgDelayMs
                << setw(14) << avgJitterMs
                << setw(14) << throughputMbps
                << endl;
        }

        cout << "========================================\n" << endl;


    }

    constexpr uint8_t
    J1939TosForPriority(uint8_t priority)
    {
        return static_cast<uint8_t>((7 - priority) << 5);
    }
    // Distinct from J1939 priority 7, which uses ToS 0x00.
    constexpr uint8_t kCameraTos = 0x10;
    constexpr uint32_t kJ1939PriorityCount = 8;
    constexpr const char* kJ1939PriorityRates[kJ1939PriorityCount] = {
        "10kbps", "10kbps", "10kbps", "10kbps",
        "10kbps", "10kbps", "10kbps", "10kbps"};

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
    SendAscFrame(Ptr<Socket> socket, AscFrame frame)
    {
        // UDP payload:
        // [CAN ID: 4 bytes][Channel: 1 byte][DLC: 1 byte][CAN data: 0-8 bytes]

        uint8_t payload[14] = {};

        payload[0] = (frame.canId >> 24) & 0xFF;
        payload[1] = (frame.canId >> 16) & 0xFF;
        payload[2] = (frame.canId >> 8) & 0xFF;
        payload[3] = frame.canId & 0xFF;

        payload[4] = static_cast<uint8_t>(frame.channel);
        payload[5] = frame.dlc;

        for (uint8_t i = 0; i < frame.dlc; ++i)
        {
            payload[6 + i] = frame.data[i];
        }

        Ptr<Packet> packet =
            Create<Packet>(payload, 6 + frame.dlc);

        socket->Send(packet);

        // static uint32_t successfulSends = 0;
        // static uint32_t failedSends = 0;

        // if (sent < 0)
        // {
        //     failedSends++;
        //     std::cerr << "ASC UDP send failed. CAN ID: 0x"
        //             << std::hex << frame.canId << std::dec
        //             << " Error: " << socket->GetErrno()
        //             << std::endl;
        // }
        // else
        // {
        //     successfulSends++;

        //     if (successfulSends <= 5)
        //     {
        //         std::cout << "ASC UDP send successful. CAN ID: 0x"
        //                 << std::hex << frame.canId << std::dec
        //                 << " Bytes: " << sent << std::endl;
        //     }
        // }
    }


    void
    RunSimulationAtRate(const string& videoRate, const vector<AscFrame>& ascFrames)
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

        // Each switch port is a separate two-device 1 Gbps CSMA segment.
        // Note: ns-3 CsmaChannel is not a full-duplex Ethernet PHY model.
        CsmaHelper csma;
        csma.SetChannelAttribute(
            "DataRate",
            DataRateValue(DataRate("1Gbps")));

        csma.SetChannelAttribute(
            "Delay",
            TimeValue(NanoSeconds(6560)));

        // Finite device-level transmit queue on each CSMA NetDevice.
        csma.SetQueue(
            "ns3::DropTailQueue<Packet>",
            "MaxSize",
            QueueSizeValue(
                QueueSize(to_string(kDeviceTxQueuePackets) + "p")));

        NetDeviceContainer terminalDevices;
        NetDeviceContainer switchPorts;

        for (uint32_t i = 0; i < terminals.GetN(); ++i)
        {
            NetDeviceContainer link =
                csma.Install(
                    NodeContainer(terminals.Get(i), switchNode.Get(0)));

            terminalDevices.Add(link.Get(0));
            switchPorts.Add(link.Get(1));
        }

        // Install Internet stack only on end systems.
        InternetStackHelper stack;
        stack.SetIpv6StackInstall(false);
        stack.Install(terminals);

        // Create the Layer-2 learning bridge.
        BridgeHelper bridge;

        bridge.SetDeviceAttribute(
            "EgressQueueLimit",
            UintegerValue(100));

        bridge.SetDeviceAttribute(
            "EgressDataRate",
            DataRateValue(DataRate("1Gbps")));

        NetDeviceContainer bridgeDevice =
            bridge.Install(switchNode.Get(0), switchPorts);

        Ptr<BridgeNetDevice> scheduledBridge =
            DynamicCast<BridgeNetDevice>(bridgeDevice.Get(0));

        // All end systems are in one IP subnet.
        Ipv4AddressHelper address;
        address.SetBase("10.1.1.0", "255.255.255.0");

        Ipv4InterfaceContainer terminalInterfaces =
            address.Assign(terminalDevices);

        // Destination is ECU 1.
        const Ipv4Address ecu1Address =
            terminalInterfaces.GetAddress(1);

        const uint16_t videoPort = 9;
        const uint16_t j1939Port = 10;

        // ---------------------------------------------------------
        // Camera traffic
        // ---------------------------------------------------------

        OnOffHelper videoApp(
            "ns3::UdpSocketFactory",
            InetSocketAddress(ecu1Address, videoPort));

        videoApp.SetAttribute(
            "OnTime",
            StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        videoApp.SetAttribute(
            "OffTime",
            StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        videoApp.SetConstantRate(DataRate(videoRate));
        videoApp.SetAttribute("PacketSize", UintegerValue(1400));
        videoApp.SetAttribute("Tos", UintegerValue(kCameraTos));

        ApplicationContainer videoApps =
            videoApp.Install(videoSource.Get(0));

        videoApps.Start(Seconds(1));
        videoApps.Stop(Seconds(10));

        PacketSinkHelper videoSink(
            "ns3::UdpSocketFactory",
            Address(InetSocketAddress(
                Ipv4Address::GetAny(),
                videoPort)));

        ApplicationContainer videoSinkApps =
            videoSink.Install(ecuNodes.Get(0));

        videoSinkApps.Start(Seconds(1));
        videoSinkApps.Stop(Seconds(10));

        // ---------------------------------------------------------
        // ASC-driven J1939 traffic
        // ---------------------------------------------------------

        // All ASC frames are delivered to ECU 1 using one UDP
        // destination port. Each transmitting ECU has its own socket.
        //
        // Socket mapping key:
        //   ASC channel, J1939 source address, J1939 priority

        map<tuple<uint32_t, uint8_t, uint8_t>, Ptr<Socket>>
            sourceSockets;

        const double replayStart = 1.0;
        const double replayScale = 1.0;

        double firstTimestamp = 0.0;
        double lastTimestamp = 0.0;

        if (!ascFrames.empty())
        {
            firstTimestamp = ascFrames.front().timestamp;
            lastTimestamp = ascFrames.back().timestamp;
        }

        const double replayDuration =
            (lastTimestamp - firstTimestamp) * replayScale;

        const double replayEnd =
            replayStart + replayDuration;

        // Install the receiving application before scheduling frames.
        PacketSinkHelper j1939Sink(
            "ns3::UdpSocketFactory",
            Address(InetSocketAddress(
                Ipv4Address::GetAny(),
                j1939Port)));

        ApplicationContainer j1939SinkApps =
            j1939Sink.Install(ecuNodes.Get(0));

        j1939SinkApps.Start(Seconds(replayStart));
        j1939SinkApps.Stop(Seconds(replayEnd + 0.1));
       

        // Schedule every CAN frame using its original ASC timestamp.
        for (const auto& frame : ascFrames)
        {
            const uint8_t priority =
                GetJ1939Priority(frame.canId);

            const uint8_t sourceAddress =
                GetJ1939SourceAddress(frame.canId);

            // Identify the transmitting flow.
            auto key = make_tuple(
                frame.channel,
                sourceAddress,
                priority);

            Ptr<Socket> socket;

            auto it = sourceSockets.find(key);

            if (it == sourceSockets.end())
            {
                // Distribute source addresses across the three
                // transmitting ECUs (ECU 2, ECU 3, ECU 4).
                //
                // This is an experimental mapping. Replace it with
                // an explicit address-to-ECU mapping if needed.
                const uint32_t sourceEcu =
                    (sourceAddress % (kNumEcus - 1)) + 1;

                socket = Socket::CreateSocket(
                    ecuNodes.Get(sourceEcu),
                    UdpSocketFactory::GetTypeId());

                // Apply the existing J1939 priority-to-ToS mapping.
                socket->SetIpTos(
                    J1939TosForPriority(priority));

                socket->Connect(
                    InetSocketAddress(ecu1Address, j1939Port));

                sourceSockets[key] = socket;
            }
            else
            {
                socket = it->second;
            }

            // Convert ASC timestamp to simulation-relative time.
            const double relativeTime =
                (frame.timestamp - firstTimestamp) * replayScale;

            const double transmissionTime =
                replayStart + relativeTime;

            // Send one UDP datagram for each CAN frame.
            Simulator::Schedule(
                Seconds(transmissionTime),
                &SendAscFrame,
                socket,
                frame);
        }

        cout << "ASC file replay:" << endl;
        cout << "  Frames scheduled: " << ascFrames.size() << endl;
        cout << "  Source flows: " << sourceSockets.size() << endl;
        cout << "  Replay start: " << replayStart << " seconds" << endl;
        cout << "  Replay duration: " << replayDuration << " seconds" << endl;
        cout << "  Replay end: " << replayEnd << " seconds" << endl;

        // ---------------------------------------------------------
        // Simulation configuration and statistics
        // ---------------------------------------------------------

        cout << "Switch model: transparent Layer-2 BridgeNetDevice" << endl;
        cout << "Forwarding: MAC learning and Ethernet frame forwarding" << endl;

        cout << "Ports: " << kNumTerminals
            << " x 1 Gbps CSMA segments (not a full-duplex PHY model)"
            << endl;

        cout << "Device TX queue limit: "
            << kDeviceTxQueuePackets
            << " packets per NetDevice" << endl;

        cout << "J1939 traffic: ASC trace replay" << endl;

        cout << "J1939 priority-to-ToS mapping: ";

        for (uint32_t priority = 0;
            priority < kJ1939PriorityCount;
            ++priority)
        {
            cout << static_cast<uint32_t>(priority)
                << "=0x" << hex
                << static_cast<uint32_t>(
                        J1939TosForPriority(priority))
                << dec;

            if (priority + 1 < kJ1939PriorityCount)
            {
                cout << ", ";
            }
        }

        cout << endl;

        cout << "Camera ToS=0x"
            << hex
            << static_cast<uint32_t>(kCameraTos)
            << dec << endl;

        cout << "Camera and ASC-driven J1939 traffic converge "
            << "on the ECU 1-facing bridge port." << endl;
        // FlowMonitor.
        FlowMonitorHelper flowmonHelper;
        Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();
        
        
        Simulator::Schedule(
            Seconds(1),
            &ns3::flowmetrics::PrintThroughputOnce,
            monitor,
            ref(lastRxBytes));

        // Allow the ASC trace to finish, including final packet reception.
        double simulationStop = replayEnd + 0.5;

        if (simulationStop < 10.5)
        {
            simulationStop = 10.5;
        }

        Simulator::Stop(Seconds(simulationStop));
        Simulator::Run();

        // ---------------------------------------------------------
        // Bridge queue statistics
        // ---------------------------------------------------------

        cout << "\nBridge egress queue statistics "
            << "(port index / queue / drops / queued):"
            << endl;

        for (uint32_t port = 0;
            port < scheduledBridge->GetNBridgePorts();
            ++port)
        {
            for (uint8_t queue = 0; queue < 9; ++queue)
            {
                cout << "  "
                    << port << " / "
                    << static_cast<uint32_t>(queue) << " / "
                    << scheduledBridge->GetEgressQueueDropCount(
                            port, queue)
                    << " / "
                    << scheduledBridge->GetEgressQueueSize(
                            port, queue)
                    << endl;
            }
        }

        // Existing complete FlowMonitor report
        flowmetrics::PrintFlowSummary(monitor);

        // Additional J1939-only report
        PrintJ1939FlowMetrics(monitor, flowmonHelper);

        Simulator::Destroy();
    }
}

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
    //cmd.AddValue("asc-file", "Input Vector ASC file", ascFile);
    cmd.Parse(argc, argv);
    vector<AscFrame> ascFrames = LoadAscFile("./scratch/CSC773Research/LiftLowerNormalOp.asc");

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
        RunSimulationAtRate(rate, ascFrames);
    }

    cout << "ASC frames loaded: " << ascFrames.size() << std::endl;


    return 0;
}

