
#include "asc-parser.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

static std::string CleanCanId(std::string id)
{
    // Vector ASC appends 'x' to extended CAN identifiers.
    if (!id.empty() && (id.back() == 'x' || id.back() == 'X'))
    {
        id.pop_back();
    }

    return id;
}


std::vector<AscFrame> LoadAscFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error(
            "Unable to open ASC file: " + filename);
    }

    std::vector<AscFrame> frames;
    std::string line;

    uint32_t candidateRecords = 0;
    uint32_t rejectedRecords = 0;

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::istringstream stream(line);

        AscFrame frame{};
        std::string canIdString;
        std::string direction;
        std::string frameType;
        std::string dlcString;

        // Read the first six fields.
        if (!(stream >> frame.timestamp
                     >> frame.channel
                     >> canIdString
                     >> direction
                     >> frameType
                     >> dlcString))
        {
            continue;
        }

        // Only process classic CAN data frames.
        if (frameType != "d")
            continue;

        candidateRecords++;

        try
        {
            canIdString = CleanCanId(canIdString);

            frame.canId = static_cast<uint32_t>(
                std::stoul(canIdString, nullptr, 16));

            unsigned long dlc = std::stoul(dlcString);

            if (dlc > 8)
            {
                rejectedRecords++;
                continue;
            }

            frame.dlc = static_cast<uint8_t>(dlc);
            frame.direction = direction;
        }
        catch (...)
        {
            rejectedRecords++;
            continue;
        }

        bool validFrame = true;

        for (uint8_t i = 0; i < frame.dlc; ++i)
        {
            std::string byteString;

            if (!(stream >> byteString))
            {
                validFrame = false;
                break;
            }

            try
            {
                unsigned long value =
                    std::stoul(byteString, nullptr, 16);

                if (value > 0xFF)
                {
                    validFrame = false;
                    break;
                }

                frame.data[i] = static_cast<uint8_t>(value);
            }
            catch (...)
            {
                validFrame = false;
                break;
            }
        }

        if (validFrame)
        {
            frames.push_back(frame);
        }
        else
        {
            rejectedRecords++;
        }
    }

    file.close();

    std::sort(frames.begin(), frames.end(),
        [](const AscFrame& a, const AscFrame& b)
        {
            return a.timestamp < b.timestamp;
        });

    std::cout << "\n========== ASC PARSER DEBUG =========="
              << "\nFile: " << filename
              << "\nCandidate CAN records: " << candidateRecords
              << "\nRejected records: " << rejectedRecords
              << "\nSuccessfully loaded: " << frames.size()
              << "\n======================================"
              << std::endl;

    return frames;
}


uint8_t GetJ1939Priority(uint32_t canId)
{
    return static_cast<uint8_t>((canId >> 26) & 0x07);
}

uint8_t GetJ1939SourceAddress(uint32_t canId)
{
    return static_cast<uint8_t>(canId & 0xFF);
}

uint32_t GetJ1939PGN(uint32_t canId)
{
    uint32_t pgn = (canId >> 8) & 0x3FFFF;
    uint8_t pf = static_cast<uint8_t>((canId >> 16) & 0xFF);

    // PDU1: PS is a destination address, not part of the PGN.
    if (pf < 240)
    {
        pgn &= 0x3FF00;
    }

    return pgn;
}
