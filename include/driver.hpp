#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "joystick.hpp"
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>

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



//Comandos del dispositivo
#define setLED_RGB 1 // Poner LED_RGB en un color determinado
#define setBearingVector 2 // Poner punto de referencia de direccion y velocidad
#define getBatteryState 3  // Leer voltaje de las baterias y enviar al master
#define startEncoderSampling 4 // comando que solicita empezar a medir el dezplazamiento en las ruedas
#define setSampleFrecuency 6 // Configurar frecuencia de muestreo del dispositivo
#define getBufferData 7 // Leer la data muestreada por el dispositivo y almacenada en el buffer
#define resetRobot  8 // Reniciar el robot

#define masterDeviceAddress 2
#define pantiltDeviceAddress 255
#define robotDeviceAddress 1 



enum robotStates
{
    idle,
    forward,
    reverse,
    stop,
    pantiltRoll,
    pantiltYaw,
    pantilt,
    readDataFromRobot,
    reset,
    powerOff

};

struct Buttons // from ps3 joystick
{

    // movimiento del robot 
    bool stop;
    bool enableForward;
    bool enableReverse;

    
    int acceleration; // -32k to 32k
    int direction ;   //  -32k to 32k

    // pantilt
    bool enableYaw;
    bool enableRoll;
    bool freeze; // guardar valores definidos
    int pantiltYaw;   // -32k-32k
    int pantiltRoll;  // -32k-32k
    

    bool reset;
    bool powerOff;

    int64_t timestampReset;
    int counterReset;


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
        void openOutputFile(string outputDirectory);
        void openJoystickDev(string device );
        void openSerialDev(string device, int baudrate);
        void run();
        bool joystickFound;
        bool serialFound;



    private:
        std::ofstream outputFilecsv;
        Joystick joystick;
        int serialPort;
        void decodeJoystickButton(JoystickEvent event);
        void stateMachine();
        void setYaw();
        void setRoll();
        void setCommandPWM();
        void setCommandEncoderSampling();
        void setCommandSampleFrecuency();
        void setCommandGetBufferData();
        void setCommandResetRobot();
        void reset();
        void dataPackage2File();

        static void alarmWakeup(int sig_num);
        

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

        static Timestamp timestamp;
        static int64_t time_stamp1, time_stamp2;
        static bool startSampling;
        static bool finishSampling;
        int countStartButton;

        static bool goToGetBufferData;
        char bufferData [1000]; // Buffer de entrada serial
        int sizeDataPackage;
        int dataEncoderLeft;
        int dataEncoderRight;
        double currentTimeSeconds;



            
};

#endif

