

#include <nanotec_driver.h>
#include <stdio.h>

Nanotec::Nanotec() : device_()
{

}

Nanotec::~Nanotec()
{
    device_.close();
}

bool Nanotec::init(std::string* port, int baudrate)
{
    try{ device_.open(port->c_str(), baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

bool Nanotec::sendMessage(char* message, unsigned int message_size, char* reply, unsigned int& reply_size)
{
    char data[NANOTEC_MSG_LEN];
    data[0] = '#';
    data[1] = '1';
    for(unsigned int i=0 ; i<message_size ; i++)
    {
        data[i+2] = message[i];
    }
    data[message_size+2] = '\r';

    device_.write(data, message_size+3);

    try{ reply_size = device_.read(data, NANOTEC_MSG_LEN, NANOTEC_TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }


    return true;
}

bool Nanotec::startMotor()
{
    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message = 'A';
    if(!sendMessage(&message, 1, reply, reply_size))
    {
        return false;
    }
    return true;
}

bool Nanotec::stopMotor()
{
    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message = 'S';
    if(!sendMessage(&message, 1, reply, reply_size))
    {
        return false;
    }
    return true;
}

bool Nanotec::setPosition(int position)
{
    if(position < NANOTEC_MIN_POSITION) position = NANOTEC_MIN_POSITION;
    if(position > NANOTEC_MAX_POSITION) position = NANOTEC_MAX_POSITION;

    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message[32];
    sprintf(message, "s=%d", position);

    if(!sendMessage(message, strlen(message), reply, reply_size))
    {
        return false;
    }
    return startMotor();
}

bool Nanotec::getPosition(int& position)
{
    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message = 'C';
    if(!sendMessage(&message, 1, reply, reply_size))
    {
        return false;
    }

    sscanf(reply, "#1C=%d", &position);
    return true;
}

bool Nanotec::startHoming()
{
    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message[] = "p=4";
    if(!sendMessage(message, 3, reply, reply_size))
    {
        return false;
    }
    return startMotor();
}

bool Nanotec::positionError()
{
    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message = '$';
    if(!sendMessage(&message, 1, reply, reply_size))
    {
        return false;
    }

    int status;
    sscanf(reply, "#001$%d", &status);
    if(status==164) return true;
    return false;
}

bool Nanotec::clearPositionError()
{
    char reply[NANOTEC_MSG_LEN];
    unsigned int reply_size;

    char message = 'D';
    if(!sendMessage(&message, 1, reply, reply_size))
    {
        return false;
    }
    return true;
}

// EOF
