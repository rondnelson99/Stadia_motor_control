#include <Bluepad32.h>

#define CONTROLLER_AXIS_ABS_RANGE 512
#define ANALOG_TRIGGER_ABS_RANGE 1024
#define BUTTON_MASK_R 1 << 5


ControllerPtr myController;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    if (myController == nullptr) {
        Serial.printf("CALLBACK: Controller is connected \n");

        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        ControllerProperties properties = ctl->getProperties();
        Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                        properties.product_id);

        myController = ctl;
        foundEmptySlot = true;
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    if (myController == ctl) {
        Serial.printf("CALLBACK: Controller disconnected \n");
        myController = nullptr;
        foundController = true;
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.

    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    init_motor_control();
}

void updateDrivetrain() {
  double yAxis = -(double) myController->axisY() / CONTROLLER_AXIS_ABS_RANGE; // the raw rating is negative when pressed forward
  double xAxis = (double) myController->axisRX() / CONTROLLER_AXIS_ABS_RANGE; 
  
  if (! (myController->buttons() & BUTTON_MASK_R)) {
    steer_drivetrain(yAxis, xAxis);
  }
  else {
    steer_drivetrain(0, 0);
  }
}

void updateBed() {
  double yAxis = -(double) myController->axisY() / CONTROLLER_AXIS_ABS_RANGE; // the raw rating is negative when pressed forward
  double xAxis = (double) myController->axisRX() / CONTROLLER_AXIS_ABS_RANGE; 
  
  if (myController->buttons() & BUTTON_MASK_R) {
    run_bed(yAxis);
  }
  else {
    run_bed(0);
  }
}

void updateArm() {
  if (myController->a()) {
    run_arm_extension(1);
  }
  else if (myController->b()) {
    run_arm_extension(-1);
  }
  else {
    run_arm_extension(0);
  }

  double arm_rotation_power = (double) (myController->throttle() - myController->brake()) / ANALOG_TRIGGER_ABS_RANGE;
  Serial.printf("Arm rotation %f ", arm_rotation_power);
  run_arm_rotation(arm_rotation_power);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    //if (dataUpdated)
    //    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    if (myController && myController->isConnected()) {
      Serial.println(myController->buttons(), BIN);
      
      updateDrivetrain();
      updateBed();
      updateArm();
      
    }

    delay(20);

}
