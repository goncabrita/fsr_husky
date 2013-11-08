

#include <cereal_port/CerealPort.h>

#define NANOTEC_MSG_LEN 128

#define NANOTEC_MIN_POSITION -100000000
#define NANOTEC_MAX_POSITION 100000000

#define NANOTEC_TIMEOUT 3

class Nanotec
{
public:
    Nanotec();
    ~Nanotec();

    bool init(std::string* port, int baudrate);
    bool setPosition(int position);
    bool getPosition(int& position);

    bool startHoming();

    bool positionError();
    bool clearPositionError();

    bool startMotor();
    bool stopMotor();

private:
    bool sendMessage(char* message, unsigned int message_size, char* reply, unsigned int& reply_size);

    cereal::CerealPort device_;
};

// EOF
