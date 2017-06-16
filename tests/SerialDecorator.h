//
// Created by Anton Matosov on 6/16/17.
//

#pragma once 

#include "SerialProtocol.h"

class SerialDecorator : SerialProtocol
{
public:

private:
    SerialProtocol *_other;
};


