#include <Bluepad32.h>
#include <ESP32Servo.h>
#include "GraspeJoints.h"

int servo2_pin = 33;
int servo3_pin = 25;
int servo4_pin = 26;
int servog_pin = 0;

int servo2_ang_init = 90;
int servo3_ang_init = 90;
int servo4_ang_init = 90;
int servog_ang_init = 0;

int servo2_ang_min = 0;
int servo3_ang_min = 0;
int servo4_ang_min = 0;
int servog_ang_min = 0;

int servo2_ang_max = 180;
int servo3_ang_max = 180;
int servo4_ang_max = 180;
int servog_ang_max = 180;

int servo2_ang_atual = servo2_ang_init;
int servo3_ang_atual = servo3_ang_init;
int servo4_ang_atual = servo4_ang_init;
int servog_ang_atual = servog_ang_init;

Servo servo2;
Servo servo3;
Servo servo4;
Servo servog;

GraspeJoints junta1(90.0,{{"min",0.0},{"max",180.0}},1.0,32);

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

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
  // R1 e L1 Funções especiais //
  if (ctl->buttons() == 0x0032) {
    Serial.println("Voltou ao dock");
    //Voltar ao dock
    //servob.write(servob_ang_init);
    //servob_ang_atual = servob_ang_init;
    servo2.write(servo2_ang_init);
    servo2_ang_atual = servo2_ang_init;
    servo3.write(servo3_ang_init);
    servo3_ang_atual = servo3_ang_init;
    servo4.write(servo4_ang_init);
    servo4_ang_atual = servo4_ang_init;
    servog.write(servo4_ang_init);
    servog_ang_atual = servog_ang_init;
    delay(100);
  }
  //== R2 trigger = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    int d_angulo_garra = map(ctl->throttle(),0,1023,45,135);
    junta1.set_angle(float(d_angulo_garra));
    Serial.println(junta1.get_angle_current());
    Serial.println(d_angulo_garra);
    //Fechar a garra
  }
  //== L2 trigger = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    int d_angulo_garra = map(ctl->brake(),0,1023,0,-20);
    Serial.println(d_angulo_garra);
    //Fechar a garra
  }

  //Checar release pra zerar o valor de d_angulo_garra
/*
//== LEFT JOYSTICK X axis==//
  if (abs(ctl->axisX())>abs(ctl->axisY())){
    if (ctl->axisX() >= 80 || ctl->axisX() <= -80) {
      if(servob_ang_atual<=180 && servob_ang_atual>=0){
        int d_angulo_base = map(ctl->axisX(),-512,512,-10,10);
        Serial.println(d_angulo_base);
        servob_ang_atual += d_angulo_base;
        if(servob_ang_atual>180){
          servob_ang_atual = 180;
        }else if(servob_ang_atual<0){
          servob_ang_atual = 0;
        }
        Serial.println(servob_ang_atual);
        servob.write(servob_ang_atual);
      }
    }
  }

*/
  
  //== LEFT JOYSTICK Y axis==//
  if (abs(ctl->axisX())<abs(ctl->axisY())){
    if (ctl->axisY() >= 80 || ctl->axisY() <= -80) {
    if(servo2_ang_atual<=180 && servo2_ang_atual>=0){
        int d_angulo_2 = map(ctl->axisY(),-512,512,-10,10);
        Serial.println(d_angulo_2);
        servo2_ang_atual += d_angulo_2;
        if(servo2_ang_atual>180){
          servo2_ang_atual = 180;
        }else if(servo2_ang_atual<0){
          servo2_ang_atual = 0;
        }
        Serial.println(servo2_ang_atual);
        //servob.write(servo2_ang_atual);
      }
    }
  }

  //== RIGHT JOYSTICK Y AXIS ==//
  if (ctl->axisRY() >= 150 || ctl->axisRY() <= -150) {
    
  } 
  if (abs(ctl->axisRX())>abs(ctl->axisRY())){
    if (ctl->axisRX() >= 80 || ctl->axisRX() <= -80) {
    if(servo3_ang_atual<=180 && servo3_ang_atual>=0){
        int d_angulo_3 = map(ctl->axisRX(),-512,512,-10,10);
        Serial.println(d_angulo_3);
        servo3_ang_atual += d_angulo_3;
        if(servo3_ang_atual>180){
          servo3_ang_atual = 180;
        }else if(servo3_ang_atual<0){
          servo3_ang_atual = 0;
        }
        Serial.println(servo3_ang_atual);
        //servob.write(servo2_ang_atual);
      }
    }
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

  /*
    Configurando Servos Motores
      - 50 Hz encontrei q tds trabalham nessa faixa, mas é sempre bom verificar data sheet
      - SG90, por testes meus (Zezé) os melhores foram min=700 e max=2350
      - MG996R, por testes os melhores são min=200 e max=3200
  */
  servo2.setPeriodHertz(50);
  servo2.attach(servo2_pin, 700, 2350);

  servo3.setPeriodHertz(50);
  servo3.attach(servo3_pin, 700, 2350);

  servo4.setPeriodHertz(50);
  servo4.attach(servo4_pin, 700, 2350);

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