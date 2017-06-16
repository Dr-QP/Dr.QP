//
// Created by Anton Matosov on 6/16/17.
//

#pragma once 

#include <string>
#import <fstream>
#include "SerialProtocol.h"

class SerialRecordingProxy: SerialProtocol
{
public:
    SerialRecordingProxy(const std::string& filename);

    void begin(const unsigned long baudRate, const uint8_t transferConfig) override;
    size_t write(uint8_t byte) override;
    bool available() override;
    uint8_t peek() override;
    uint8_t read() override;

private:
    std::ofstream _file;
};


