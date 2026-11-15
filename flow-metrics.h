#ifndef FLOW_METRICS_H
#define FLOW_METRICS_H

#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/traffic-control-module.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <map>

namespace ns3
{
namespace flowmetrics
{

inline void
PrintThroughputOnce(Ptr<FlowMonitor> monitor, std::map<uint32_t, uint64_t>& lastRxBytes)
{
    auto flowStats = monitor->GetFlowStats();
    for (const auto& flow : flowStats)
    {
        const auto& stats = flow.second;
        const uint64_t currentRx = stats.rxBytes;
        const uint64_t deltaRx = currentRx - lastRxBytes[flow.first];
        const double throughputMbps = (deltaRx * 8.0) / 1e6;
        //Testing123

        std::cout << "Time " << Simulator::Now().As(Time::S) << "s | flow " << flow.first
                  << " | txBytes=" << stats.txBytes << " | rxBytes=" << stats.rxBytes
                  << " | lostPackets=" << stats.lostPackets
                  << " | intervalThroughput=" << throughputMbps << " Mbps" << std::endl;

        lastRxBytes[flow.first] = currentRx;
    }

    if (Simulator::Now() < Seconds(10))
    {
        Simulator::Schedule(Seconds(1), &PrintThroughputOnce, monitor, std::ref(lastRxBytes));
    }
}

inline void
PrintQueueStats(Ptr<QueueDisc> queueDisc)
{
    if (!queueDisc)
    {
        return;
    }

    const auto& stats = queueDisc->GetStats();
    std::cout << "Time " << Simulator::Now().As(Time::S) << "s | queue="
              << queueDisc->GetInstanceTypeId().GetName() << " | enqueued="
              << stats.nTotalEnqueuedPackets << " | sent=" << stats.nTotalSentPackets
              << " | dropped=" << stats.nTotalDroppedPackets << " | backlog="
              << queueDisc->GetNPackets() << std::endl;

    if (Simulator::Now() < Seconds(10))
    {
        Simulator::Schedule(Seconds(1), &PrintQueueStats, queueDisc);
    }
}

inline void
PrintFlowSummary(Ptr<FlowMonitor> monitor)
{
    auto flowStats = monitor->GetFlowStats();
    std::cout << "\n=== FlowMonitor summary ===" << std::endl;
    for (const auto& flow : flowStats)
    {
        const auto& stats = flow.second;
        const double duration = (stats.timeLastRxPacket - stats.timeFirstTxPacket).GetSeconds();
        const double throughputMbps = (stats.rxBytes * 8.0) / std::max(1e-9, duration * 1e6);
        const double avgDelaySeconds =
            stats.rxPackets > 0 ? (stats.delaySum.GetSeconds() / stats.rxPackets) : 0.0;

        std::cout << "Flow " << flow.first << " | txBytes=" << stats.txBytes
                  << " | rxBytes=" << stats.rxBytes << " | packetLoss=" << stats.lostPackets
                  << " | throughput=" << throughputMbps << " Mbps"
                  << " | avgDelay=" << avgDelaySeconds << " s"
                  << " | minDelay=" << stats.minDelay.GetSeconds() << " s"
                  << " | maxDelay=" << stats.maxDelay.GetSeconds() << " s" << std::endl;
    }
}

} // namespace flowmetrics
} // namespace ns3

#endif /* FLOW_METRICS_H */
