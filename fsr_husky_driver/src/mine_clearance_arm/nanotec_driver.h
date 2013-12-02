

#include <cereal_port/CerealPort.h>

#define NANOTEC_MSG_LEN 128

#define NANOTEC_MIN_POSITION -100000000
#define NANOTEC_MAX_POSITION 100000000

#define NANOTEC_TIMEOUT 3000

#define NANOTEC_LEFT    0
#define NANOTEC_RIGHT   1

#define NANOTEC_STATUS_ZERO     163
#define NANOTEC_STATUS_LIMIT    164

class Nanotec
{
public:
    Nanotec();
    ~Nanotec();

    bool init(std::string* port, int baudrate);

    bool setPositionMode();
    bool setDirection(unsigned int direction);

    bool setPosition(int position);
    bool getPosition(int& position);

    bool startHoming();

    int getStatus();
    bool clearPositionError();

    bool startMotor();
    bool stopMotor();

private:
    bool sendMessage(char* message, unsigned int message_size, char* reply, unsigned int& reply_size);

    cereal::CerealPort device_;
};

// EOF
