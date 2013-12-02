

#include <cereal_port/CerealPort.h>

#define JRK_MIN_POSITION 0
#define JRK_MAX_POSITION 4095

#define JRK_TIMEOUT 3

#define JRK_21V3_CURRENT_UNIT_VALUE     38
#define JRK_21V12_CURRENT_UNIT_VALUE    149

class JRK
{
public:
    JRK();
    ~JRK();

    bool init(std::string* port, int baudrate);
    bool setPosition(int position);
    bool getPosition(int& position);
    bool getCurrent(double& current);

private:
    bool getValue(int& value);

    cereal::CerealPort device_;
};

// EOF
