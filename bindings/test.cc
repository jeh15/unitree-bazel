#include "lowlevelapi.h"
#include "lowlevelapi_types.h"


int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    MotorController controller;
    controller.init();

    auto start = std::chrono::high_resolution_clock::now();

    // Control Rate:
    const uint8_t dt = 20;
    lowleveltypes::MotorCommand motor_command;

    // Set Motor Command:
    motor_command.stiffness = {
        60.0, 60.0, 60.0,
        60.0, 60.0, 60.0,
        60.0, 60.0, 60.0,
        60.0, 60.0, 60.0
    };
    motor_command.damping = {
        5.0, 5.0, 5.0,
        5.0, 5.0, 5.0,
        5.0, 5.0, 5.0,
        5.0, 5.0, 5.0
    };
    motor_command.kp = {
        10.0, 10.0, 10.0,
        10.0, 10.0, 10.0,
        10.0, 10.0, 10.0,
        10.0, 10.0, 10.0
    };
    motor_command.kd = {
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0
    };

    // Update Controller Setpoints and Values:
    controller.update_command(motor_command);

    while (true) {
        auto elapsed_time = std::chrono::high_resolution_clock::now() - start;
        // Send MotorController Command:

        // Get state:
        std::string key = "Front_Right_Abduction";
        controller.get_state(key);

        // Sleep for dt:
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));

        // Run for 10 Seconds:
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count() > 60.0)
            break;
        
    }

    return 0;
}
