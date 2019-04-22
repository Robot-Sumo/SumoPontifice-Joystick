#include "driver.hpp"
#include <unistd.h>
#include <wiringSerial.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

// public
Driver::Driver()
{
    robotState = robotStates::reset;
    joystickFound = false;
    serialFound = false;
    reset();
}

void Driver::reset()
{
    robotButton.stop = false;
    robotButton.enableForward = false;
    robotButton.enableReverse = false;
    robotButton.acceleration = -32767;
    robotButton.direction = 0;
    robotButton.enableRoll = false;
    robotButton.enableYaw = false;
    robotButton.pantiltYaw = 0;
    robotButton.pantiltRoll = 0;
    robotButton.freeze = false;
    robotButton.reset = false;
    robotButton.powerOff = false;

    pwmForward = 1;
    pwmRight = 0;
    pwmLeft = 0;
    yaw = yaw_90;
    roll = roll_0;

    setPWM();
    usleep(1000);
    setYaw();
    usleep(20000);
    setRoll();
}


void Driver::openJoystickDev(string device)
{
    joystick.openPath(device);
    joystickFound = joystick.isFound();
  
}

void Driver::openSerialDev(string device, int baudrate )
{
    serialPort = serialOpen (device.c_str(), baudrate);
    serialFound = (serialPort != -1) ;

}

// private
void Driver::decodeJoystickButton(JoystickEvent event)
{
   if (event.isButton())
    {
       
        switch (event.number)
        {
            case Cross:
                robotButton.enableForward = (event.value == 1);   
                break;
            case Circle:
                break;
            case Triangle:
                robotButton.enableReverse = (event.value == 1);
                break;
            case Rectangle:
                robotButton.stop = (event.value == 1);
                break;
            case L1:
                robotButton.freeze = (event.value == 1);
                break;
            case R1:
                break;
            case L2:
                break;
            case R2:
                break;
            case Select:
                break;
            case Start:
                robotButton.reset = (event.value==1);
                break;
            case PS:
                robotButton.powerOff = (event.value==1);
                break;
            case L3_button:
                break;
            case R3_button:
                break;
            case Up:
                robotButton.enableRoll = (event.value==1);
                break;
            case Down:
                //robotButton.enableRoll = (event.value==1);
                break;
            case Left:
                robotButton.enableYaw = (event.value==1);
                break;
            case Right:
                break;
            default:
                break;
         }

    }
    else if (event.isAxis())
    {   
        switch (event.number)
        {
            case R3_x_axis:
                robotButton.pantiltYaw = event.value;
                break;
            case R3_y_axis:
                robotButton.pantiltRoll = event.value;
            case L3_x_axis:
                robotButton.direction = event.value;
                break;
            case L3_y_axis:
                break;
            case R2_x_axis:
                robotButton.acceleration = event.value;
                break;
            case L2_x_axis:
                break;
            default:
                break;

        }

    }
}



void Driver::run()
{
    // Attempt to sample an event from the joystick
    Timestamp timestamp;
    int64_t time_stamp1, time_stamp2;

    stateMachine();

    JoystickEvent event;
    if (joystick.sample(&event))
    {
        decodeJoystickButton(event);
        //time_stamp2 = timestamp.getNanoSecs();
        stateMachine();
        
        
    }
    


    if(lastPwmForward != pwmForward ||  lastPwmRight != pwmRight|| 
    lastPwmLeft != pwmLeft ||
    lastYaw != yaw ||
    lastRoll != roll )
    {
        cout << "direction = " <<(pwmForward == 1? "forward" : "reverse")
        << " pwmRight = " << pwmRight
        << " pwmLeft = " << pwmLeft
        << " yaw = " << yaw
        << " roll = " << roll
        <<endl;
    }
  

    lastPwmForward = pwmForward;
    lastPwmRight = pwmRight;
    lastPwmLeft = pwmLeft;
    lastYaw = yaw;
    lastRoll = roll;
  
    usleep(5000);

}


void Driver::stateMachine()
{  

    switch (robotState)
    {
        case robotStates::idle:

            // Desplazamiento
            if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else if(robotButton.reset) robotState = robotStates::reset;
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw; // No está pulsado ningun boton de Desplazamiento ni el reset
            else robotState = robotStates::idle;
            //cout << "idle" <<endl;
    
            
            break;
        case robotStates::forward:

            pwmForward = 1;
            pwmModule = remap(-32767, 32767, 0, 255, robotButton.acceleration);

            if (robotButton.direction >=0) //Cruzar a la derecha, Reducir pwm rueda derecha
            {
                pwmRight = remap2(pwmModule, robotButton.direction);
                pwmLeft = pwmModule;
                
            }
            else // Cruzar a la izquierda, Reducir pwm rueda izquierdelsea
            {    
                pwmLeft = remap2(pwmModule, robotButton.direction);
                pwmRight = pwmModule;
                
            }
            setPWM();

            // Pantilt
            if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw;
            else robotState = robotStates::idle;
            //cout << "forward" << endl;

            break;
        case robotStates::reverse:
            pwmForward = 0;

            pwmModule = remap(-32767, 32767, 0, 255, robotButton.acceleration);

            if (robotButton.direction >=0) //Cruzar a la derecha, Reducir pwm rueda derecha
            {
                pwmRight = remap2(pwmModule, robotButton.direction);
                pwmLeft = pwmModule;
                
            }
            else // Cruzar a la izquierda, Reducir pwm rueda izquierdelsea
            {    
                pwmLeft = remap2(pwmModule, robotButton.direction);
                pwmRight = pwmModule;
                
            }
            setPWM();

            // Pantilt
            if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw;
            else robotState = robotStates::idle;
            

            //cout << "reverse" << endl;
            break;
        case robotStates::stop:
            pwmForward = 1;
            pwmModule = 0;
            pwmLeft = 0;
            pwmRight = 0;
            setPWM();
            
            if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw;
            else robotState = robotStates::idle;
            //cout << "stop" << endl;
            break;

        case robotStates::pantilt:
            yaw =  remap(-32767, 32767, 0, 255, robotButton.pantiltYaw);
            roll = remap(-32767, 32767, 0, 255, robotButton.pantiltRoll);

            setYaw();
            usleep(10000);
            setRoll();

            if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else robotState = robotStates::stop;
            //cout << "pantilt" << endl;
            break;

        case robotStates::pantiltYaw:
            yaw =  remap(-32767, 32767, 0, 255, robotButton.pantiltYaw);

            setYaw();

            if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else robotState = robotStates::stop;
            //cout << "pantiltYaw" << endl;
            break;

        case robotStates::pantiltRoll:
            roll =  remap(-32767, 32767, 0, 255, robotButton.pantiltRoll);

            setRoll();
            
            if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else robotState = robotStates::stop;
            //cout << "pantiltRoll" << endl;
            break;
        
        case robotStates::reset:
            pwmForward = 1;
            pwmRight = 0;
            pwmLeft = 0;
            yaw = yaw_90;
            roll = roll_0;
            setPWM();
            usleep(10000);
            setYaw();
            usleep(10000);
            setRoll();

            robotState = robotStates::idle;
            //cout << "reset" << endl;

            break;

        case robotStates::powerOff:
            serialClose(serialPort);
            joystick.closeDevice();
            //string powerDown = "sudo shutdown -h now";
            // Delete old directories
            system("sudo shutdown -h now");
            exit(1);
            break;

        default:
            break;

    }
}

void Driver::setYaw()
{

    char write_buffer[] = {255, 1, yaw_0 }; 
    if (yaw>yaw_180)
    {
        write_buffer[2] = yaw_180;
    }
    else if (yaw<yaw_0)
    {
        write_buffer[2] = yaw_0;
    }
    else
    {
        write_buffer[2] = yaw;
        
    }

    write(serialPort , write_buffer, sizeof(write_buffer));
    

}



void Driver::setRoll()
{
    char write_buffer[] = {255, 2, roll_min }; 
    if (roll>roll_90)
    {
        write_buffer[2] = roll_90;
    }
    else if (roll<roll_min)
    {
        write_buffer[2] = roll_min;
    }
    else
    {
        write_buffer[2] = roll;
    }
    write(serialPort, write_buffer,sizeof(write_buffer));
 
}


void Driver::setPWM()
{
    char address = 1;
    char command = 1; // set pwm
    char lastchar = '\n';
    char write_buffer[] = {address, command, char(pwmLeft), char(pwmRight), char(pwmForward), lastchar };  
    if(pwmRight>=0 && pwmRight <=255 && pwmLeft>=0 && pwmLeft<=255)
    {
        write(serialPort ,write_buffer,sizeof(write_buffer));

    }

}


int Driver::remap(int lowest, int highest, int newLowest, int newHighest, int analogValue)
{
    float newValue = 1.0*(analogValue-lowest)/(highest-lowest); // 0 to 1.0

    float map = newValue*(newHighest-newLowest)+newLowest;

    return int(map);
}

int Driver::remap2(int acceleration, int analogValue)
{
    float newValue = acceleration*(1.0-abs(analogValue)/(32767.0)); // 1.0 to 0.0


    return int(newValue);
}


