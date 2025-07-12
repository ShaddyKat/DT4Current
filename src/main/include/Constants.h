#pragma once

namespace canid {
    constexpr int DRIVEBASE_FRONT_RIGHT_DRIVE = 10; 
    constexpr int DRIVEBASE_FRONT_RIGHT_TURN = 8;
    constexpr int DRIVEBASE_FRONT_RIGHT_ENCODER = 9; 

    constexpr int DRIVEBASE_FRONT_LEFT_DRIVE = 7;
    constexpr int DRIVEBASE_FRONT_LEFT_TURN = 5;
    constexpr int DRIVEBASE_FRONT_LEFT_ENCODER = 6;
        
    constexpr int DRIVEBASE_BACK_RIGHT_DRIVE = 13; 
    constexpr int DRIVEBASE_BACK_RIGHT_TURN = 11;
    constexpr int DRIVEBASE_BACK_RIGHT_ENCODER = 12; 

    constexpr int DRIVEBASE_BACK_LEFT_DRIVE = 3; 
    constexpr int DRIVEBASE_BACK_LEFT_TURN = 1;
    constexpr int DRIVEBASE_BACK_LEFT_ENCODER = 2; 
}

namespace dio {
    constexpr int ENDEFFECTOR_LINEBREAK_VERTICAL = 0;
    
    //blue btn
    constexpr int BRAKE_COAST_BUTTON = 1;
    // yellow btn
    constexpr int ARM_ZERO_BUTTON = 2;
}

namespace pwm {
    constexpr int LED = 0;
}

namespace math {
    constexpr double pi = 3.14159265358979323846;
}