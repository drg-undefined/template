#pragma once
#include <frc/Joystick.h>

class OI
{
    public: 
        double getValue(void){
            return joystick.GetRawAxis(0);
        }
    
    private:
        //Controller Port
        #define DRIVE_USB_PORT              0
        frc::Joystick joystick{DRIVE_USB_PORT};

};