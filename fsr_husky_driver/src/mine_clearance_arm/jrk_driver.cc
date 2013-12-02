

#include "jrk_driver.h"
#include <stdio.h>

JRK::JRK() : device_()
{

}

JRK::~JRK()
{
    device_.close();
}

bool JRK::init(std::string* port, int baudrate)
{
    try{ device_.open(port->c_str(), baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

bool JRK::getValue(int& value)
{
    char data = 0xA7;
    device_.write(&data);

    char reply[2];
    try{ device_.read(reply, 2, JRK_TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    value = reply[0] + 256*reply[1];
    return true;
}

bool JRK::setPosition(int position)
{
    printf("Position %d\n", position);

    if(position < JRK_MIN_POSITION) position = JRK_MIN_POSITION;
    if(position > JRK_MAX_POSITION) position = JRK_MAX_POSITION;

    char data[2];
    data[0] = 0xC0 + (position & 0x1F);
    data[1] = (position >> 5) & 0x7F;

    device_.write(data, 2);

    return true;
}

bool JRK::getPosition(int& position)
{
    return getValue(position);
}

bool JRK::getCurrent(double& current)
{
    int reply;

    if(!getValue(reply))
    {
        return false;
    }

    current = reply*JRK_21V3_CURRENT_UNIT_VALUE/1000.0;

    return true;
}

// EOF
