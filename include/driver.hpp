#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "joystick.hpp"
#include <string>
#include <chrono>

using namespace std;
using namespace std::chrono;

// PS3 Joystick Buttons
#define Cross 0
#define Circle 1
#define Triangle 2
#define Rectangle 3
#define L1 4
#define R1 5
#define L2 6
#define R2 7
#define Select 8
#define Start 9
#define PS 10
#define L3_button 11
#define R3_button 12
#define Up 13
#define Down 14
#define Left 15
#define Right 16
#define L3_x_axis 0
#define L3_y_axis 1
#define R3_x_axis 3
#define R3_y_axis 4
#define R2_x_axis 5
#define L2_x_axis 2



enum robotStates
{
    idle,
    forward,
    reverse,
    stop,
    pantiltRoll,
    pantiltYaw,
    pantilt,
    reset,
    powerOff

};

struct Buttons // from ps3 joystick
{

    // movimiento del robot 
    bool stop;
    bool enableForward;
    bool enableReverse;

    
    int acceleration; // -32k-32k
    int direction ;   //  -32k-32k

    // pantilt
    bool enableYaw;
    bool enableRoll;
    bool freeze; // guardar valores definidos
    int pantiltYaw;   // -32k-32k
    int pantiltRoll;  // -32k-32k
    

    bool reset;
    bool powerOff;


};


class Timestamp{


public:
    Timestamp(){}

    int64_t getNanoSecs(){
    
    nanoseconds ns = duration_cast< nanoseconds >(
        system_clock::now().time_since_epoch()
    );
    return ns.count();
    }



};

class Driver
{
    public:

        Driver();
        
        void openJoystickDev(string device );
        void openSerialDev(string device, int baudrate);
        void run();
        bool joystickFound;
        bool serialFound;



    private:
        Joystick joystick;
        int serialPort;
        void decodeJoystickButton(JoystickEvent event);
        void stateMachine();
        void setYaw();
        void setRoll();
        void setPWM();
        void reset();

        int remap(int lowest, int highest, int newLowest, int newHighest, int analogValue);
        int remap2(int acceleration, int analogValue);


        // Variables actuales
        int pwmForward;
        int pwmRight;
        int pwmLeft;
        int pwmModule;
        int yaw;
        int roll;

        int lastPwmForward;
        int lastPwmRight;
        int lastPwmLeft;
        int lastPwmModule;
        int lastYaw;
        int lastRoll;


        // Calibracion pantilt
        const uint8_t yaw_180 =228;
        const uint8_t yaw_0 = 36;
        const uint8_t yaw_90 = 96;

        const uint8_t roll_90 = 245;
        const uint8_t roll_0 = 131;
        const uint8_t roll_min = 100;  // mini

        Buttons robotButton;
        int robotState;
        
        
    
};

#endif

