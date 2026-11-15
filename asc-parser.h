#ifndef ASC_PARSER_H
#define ASC_PARSER_H

#include <cstdint>
#include <string>
#include <vector>

struct AscFrame
{
    double timestamp;
    uint32_t channel;
    uint32_t canId;
    std::string direction;
    uint8_t dlc;
    uint8_t data[8];
};

// Read classic CAN frames from a Vector ASC file.
std::vector<AscFrame> LoadAscFile(const std::string& filename);

// J1939 identifier helpers.
uint8_t GetJ1939Priority(uint32_t canId);
uint8_t GetJ1939SourceAddress(uint32_t canId);
uint32_t GetJ1939PGN(uint32_t canId);

#endif // ASC_PARSER_H