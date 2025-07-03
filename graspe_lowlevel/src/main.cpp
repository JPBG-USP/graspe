#include <Bluepad32.h>
#include <ESP32Servo.h>
#include "GraspeManipulator.h"

GraspeManipulator graspe_manipulator;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

bool joint_flag = false;
bool inspection_flag = false;

Servo garra;
int garra_max = 170;
int garra_min = 70;
int angulo_garra = 170;
unsigned long last_press = 0;

float degrees_to_rad(float input){
  return input*M_PI/180;
}
float rad_to_degrees(float input){
  return input*180/M_PI;
}

void cylindrical_control(ControllerPtr ctl){
  //delta_pos vector saves the change in position of the manipulator for each loop, at the end
  //of the loop it is passed in the set_pose method.
  graspe::CylindricalCoord delta_pos = {0.0, 0.0, 0.0, 0.0};
  unsigned long tempo = millis();
  // == R1 e L1 Special functions == //

  // = Reset Manipulator - - - B = //
  if (ctl->buttons() == 0x0032) {
    graspe_manipulator.reset_manipulator();
    garra.write(170);
    delay(100);
  }
  // = Toggle inspection mode - - -  = //
  if(ctl -> buttons() == 0x0034){
    inspection_flag = !inspection_flag;  
  }
  // = Change control mode = //
  if(ctl -> buttons() == 0x0038){
    if((millis()-last_press)>3000){
      joint_flag = !joint_flag;
      //Serial.println("Foi da cilindrica pra joint");
      last_press = millis();
    }
  }

  // == Joystick and trigger inspections == //

  //== R2 trigger = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    int d_angulo_garra = map(ctl->throttle(),0,1023,0,-10);
    if(d_angulo_garra+angulo_garra>=garra_max){
      angulo_garra=garra_max;
    }else if(d_angulo_garra+angulo_garra<=garra_min){
      angulo_garra=garra_min;
    }else{
      angulo_garra+=d_angulo_garra;
    }
    garra.write(angulo_garra);
  }
  //== L2 trigger = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    int d_angulo_garra = map(ctl->brake(),0,1023,0,10);
    if(d_angulo_garra+angulo_garra>=garra_max){
      angulo_garra=garra_max;
    }else if(d_angulo_garra+angulo_garra<=garra_min){
      angulo_garra=garra_min;
    }else{
      angulo_garra+=d_angulo_garra;
    }
    garra.write(angulo_garra);
  }

//== LEFT JOYSTICK X axis - - - Theta ==//
  int d_base_angle = 0;
  if (abs(ctl->axisX())>abs(ctl->axisY())){
    if (ctl->axisX() >= 80 || ctl->axisX() <= -80) {
        int d_base_angle = map(ctl->axisX(),-512,512,5,-5);
        delta_pos[0] = degrees_to_rad(float(d_base_angle));
    }
  }
  
  //== LEFT JOYSTICK Y axis - - - R ==//
  int d_pos_r = 0;
  if (abs(ctl->axisX())<abs(ctl->axisY())){
    if (ctl->axisY() >= 80 || ctl->axisY() <= -80) {
      int d_pos_r = map(ctl->axisY(),-512,512,5,-5);
      delta_pos[1] = d_pos_r*0.1;
    }
  }
  
  //== RIGHT JOYSTICK Y AXIS - - - Z==//
  int d_pos_z = 0;
  if(abs(ctl->axisRX())<abs(ctl->axisRY())){
    if (ctl->axisRY() >= 80 || ctl->axisRY() <= -80) {
      int d_pos_z = map(ctl->axisRY(),-512,512,5,-5);
      delta_pos[2] = d_pos_z*0.1;
    } 
  }
  
  //== RIGHT JOYSTICK Y AXIS - - - Approach ==//
  int d_approach_angle = 0;
  if (abs(ctl->axisRX())>abs(ctl->axisRY())){
    if (ctl->axisRX() >= 80 || ctl->axisRX() <= -80) {
      int d_approach_angle = map(ctl->axisRX(),-512,512,5,-5);
      delta_pos[3] = degrees_to_rad(float(d_approach_angle));
    }
  }

  if(!graspe_manipulator.set_pose(delta_pos)){
    ctl->playDualRumble(0,250,255,255);
    Serial.println("Falhou no set_pose");
  }
  //Inspection flag changes the Serial output
  if(inspection_flag){
    Serial.println("=== Inspection mode active ===");
    Serial.println("Joint 1:");
    Serial.printf("d_base_angle (degrees) = %d\n",d_base_angle);
    Serial.printf("d_base_angle (radians) = %d\n",degrees_to_rad(float(d_base_angle)));
    Serial.println("Joint 2:");
    Serial.printf("d_pos_r (cm) = %d\n",d_pos_r);
    Serial.println("Joint 3:");
    Serial.printf("d_pos_z (cm) = %d\n",d_pos_z);
    Serial.println("Joint 4:");
    Serial.printf("d_approach_angle (degrees) = %d\n",d_approach_angle);
    Serial.printf("d_approach_angle (radians) = %d\n",degrees_to_rad(float(d_approach_angle)));
    Serial.println("General information");
    Serial.printf("d_pos vector: {%f,%f,%f,%f}",graspe_manipulator.endeffector_pose[0],
                                                graspe_manipulator.endeffector_pose[1],
                                                graspe_manipulator.endeffector_pose[2],
                                                graspe_manipulator.endeffector_pose[3]);
    Serial.println(joint_flag?"joint control active":"cylindrical control active");
  }else{
    //Prints joint states for the Peter Corke visualization

   
   Serial.printf("cylindrical %f/%f/%f/%f\n",
      graspe_manipulator.joint1.get_angle_current(),
      graspe_manipulator.joint2.get_angle_current(),
      graspe_manipulator.joint3.get_angle_current(),
      graspe_manipulator.joint4.get_angle_current()
    );
  
  }
  //Serial.println("================================================");
}

void joint_control(ControllerPtr ctl){
  
  // delta_pos vector saves the change in position of the manipulator for each loop, at the end
  // of the loop it is passed in the set_pose method.
  graspe::JointStates delta_joint = {0.0, 0.0, 0.0, 0.0};
  unsigned long tempo = millis();
  // == R1 e L1 Special functions == //

  // = Reset Manipulator = //
  if (ctl->buttons() == 0x0032) {
    graspe_manipulator.reset_manipulator();
    garra.write(170);
    delay(100);
  }
  // = Toggle inspection mode = //
  if(ctl -> buttons() == 0x0034){
    inspection_flag = !inspection_flag;  
  }
  // = Change control mode = //
  if(ctl -> buttons() == 0x0038){
    if((millis()-last_press)>3000){
      joint_flag = !joint_flag;
      //Serial.println("Foi da joint pra cilindrica");
      last_press = millis();
    }
  }

  // == Joystick and trigger inspections == //

  //== R2 trigger = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    int d_angulo_garra = map(ctl->throttle(),0,1023,0,-10);
    if(d_angulo_garra+angulo_garra>=garra_max){
      angulo_garra=garra_max;
    }else if(d_angulo_garra+angulo_garra<=garra_min){
      angulo_garra=garra_min;
    }else{
      angulo_garra+=d_angulo_garra;
    }
    garra.write(angulo_garra);
  }
  //== L2 trigger = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    int d_angulo_garra = map(ctl->brake(),0,1023,0,10);
    if(d_angulo_garra+angulo_garra>=garra_max){
      angulo_garra=garra_max;
    }else if(d_angulo_garra+angulo_garra<=garra_min){
      angulo_garra=garra_min;
    }else{
      angulo_garra+=d_angulo_garra;
    }
    garra.write(angulo_garra);
  }


//== LEFT JOYSTICK X axis - - - Theta ==//
  int d_angle_1 = 0;
  if (abs(ctl->axisX())>abs(ctl->axisY())){
    if (ctl->axisX() >= 80 || ctl->axisX() <= -80) {
        d_angle_1 = map(ctl->axisX(),-512,512,5,-5);
        delta_joint[0] = degrees_to_rad((float)d_angle_1);
        //graspe_manipulator.joint1.set_angle(graspe_manipulator.joint1.get_angle_current()+degrees_to_rad((float)d_angle_1));
    }
  }
  
  //== LEFT JOYSTICK Y axis - - - R ==//
  int d_angle_2 = 0;
  if (abs(ctl->axisX())<abs(ctl->axisY())){
    if (ctl->axisY() >= 80 || ctl->axisY() <= -80) {
      d_angle_2 = map(ctl->axisY(),-512,512,5,-5);
      delta_joint[1] = degrees_to_rad((float)d_angle_2);
      //graspe_manipulator.joint2.set_angle(graspe_manipulator.joint2.get_angle_current()+degrees_to_rad((float)d_angle_2));
    }
  }
  
  //== RIGHT JOYSTICK Y AXIS - - - Z==//
  int d_angle_3 = 0;
  if(abs(ctl->axisRX())<abs(ctl->axisRY())){
    if (ctl->axisRY() >= 80 || ctl->axisRY() <= -80) {
      d_angle_3 = map(ctl->axisRY(),-512,512,5,-5);
      delta_joint[2] = degrees_to_rad((float)d_angle_3);
      //graspe_manipulator.joint3.set_angle(graspe_manipulator.joint3.get_angle_current()+degres_to_rad((float)d_angle_3));
    } 
  }
  
  //== RIGHT JOYSTICK X AXIS ==//
  int d_angle_4 = 0;
  if (abs(ctl->axisRX())>abs(ctl->axisRY())){
    if (ctl->axisRX() >= 80 || ctl->axisRX() <= -80) {
      d_angle_4 = map(ctl->axisRX(),-512,512,-5,+5);
      delta_joint[3] = degrees_to_rad((float)d_angle_4);
      //graspe_manipulator.joint4.set_angle(graspe_manipulator.joint4.get_angle_current()+degrees_to_rad((float)d_angle_4));
    }
  }

  if(!graspe_manipulator.set_joint_pose(delta_joint)){
    ctl->playDualRumble(0,250,255,255);
    Serial.println("Falhou no set_joint_pose");
  }

  //Inspection flag changes the Serial output
  if(inspection_flag){
    Serial.println("=== Inspection mode active ===");
    Serial.println("Joint 1:");
    Serial.printf("d_angle_1 (degrees) = %d\n",d_angle_1);
    Serial.printf("d_angle_1 (radians) = %d\n",degrees_to_rad(float(d_angle_1)));
    Serial.println("Joint 2:");
    Serial.printf("d_angle_2 (degrees)= %d\n",d_angle_2);
    Serial.printf("d_angle_2 (radians) = %d\n",degrees_to_rad(float(d_angle_2)));
    Serial.println("Joint 3:");
    Serial.printf("d_angle_3 (degrees) = %d\n",d_angle_3);
    Serial.printf("d_angle_3 (radians) = %d\n",degrees_to_rad(float(d_angle_3)));
    Serial.println("Joint 4:");
    Serial.printf("d_angle_4 (degrees) = %d\n",d_angle_4);
    Serial.printf("d_angle_4 (radians) = %d\n",degrees_to_rad(float(d_angle_4)));
    Serial.println("General information");
    Serial.printf("Joint states (degrees): {%f,%f,%f,%f\n}", rad_to_degrees(graspe_manipulator.joint1.get_angle_current()),
                                                             rad_to_degrees(graspe_manipulator.joint2.get_angle_current()),
                                                             rad_to_degrees(graspe_manipulator.joint3.get_angle_current()),
                                                             rad_to_degrees(graspe_manipulator.joint4.get_angle_current()));
    Serial.println(joint_flag?"joint control active":"cylindrical control active");
  }else{
    //Prints joint states for the Peter Corke visualization

   
    Serial.printf("joint %f/%f/%f/%f\n",
      graspe_manipulator.joint1.get_angle_current(),
      graspe_manipulator.joint2.get_angle_current(),
      graspe_manipulator.joint3.get_angle_current(),
      graspe_manipulator.joint4.get_angle_current()
    );
   
  }
  //Serial.println("================================================");
}

// ------ Beggining of code from Bluepad32.h examples ------ //

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle button
  ctl->miscButtons()  // bitmask of pressed "misc" buttons
  )
  ;
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // Flag for joint control mode (AKA debug mode)
  if(joint_flag==false){
    cylindrical_control(ctl);
    //Serial.println("Modo cilindrico");
    //digitalWrite(2,LOW);
  }
  if(joint_flag==true){
    joint_control(ctl);
    //Serial.println("Modo JJ");
    //digitalWrite(2,HIGH);
  }
  //dumpGamepad(ctl);
}


void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);

  //Coisas que tem a ver com o controle remoto
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

  garra.setPeriodHertz(50);
  garra.attach(27, 700, 2350);
  garra.write(170);

  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);
  delay(500);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  delay(50);
}