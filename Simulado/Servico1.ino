#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct {

  bool isConnectedToTheNetwork = false;
  byte numSsid;

  int findSsidIndex(String targetSsid) {
    // Obtém o número total de redes salvas
    int numNetworks = WiFi.scanNetworks();

    // Itera por todas as redes salvas
    for (int i = 0; i < numNetworks; i++) {

      String currentSsid = WiFi.SSID(i);

      if (currentSsid.equals(targetSsid)) {
        return i;
      }
    }

    // Se a rede não for encontrada, retorna -1
    return -1;
  }

  //Conecta a Rede Por Meio Do Indice Fornecido Pelo metodo findLocalNetworks
  bool connectToTheNetworkBySSIDIndex(int ssidIndex, String password = "") {

    String ssid = WiFi.SSID(ssidIndex);

    if (WiFi.encryptionType(ssidIndex) == WIFI_AUTH_OPEN) {
      
      WiFi.begin(ssid); 
    
    } else {

      WiFi.begin(ssid, password); 
    
    } 
    
    Serial.println("Conectando...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("\nSucesso");
    
    return true;
  }

  bool connectToTheNetworkBySSIDName(String ssid, String password = ""){

    int ssidIndex = findSsidIndex(ssid);
    if(ssidIndex < 0) return false;

    connectToTheNetworkBySSIDIndex(ssidIndex, password);

    return true;

  }

    //Procura Pelas redes Identificadas no Local
  void findLocalNetworks() {

    WiFi.disconnect(); // limpa conexões anteriores
    isConnectedToTheNetwork = false;

    numSsid = WiFi.scanNetworks();

    // print the list of networks seen:
    Serial.print("Lista de SSID:");
    Serial.println(numSsid);
    
    // print the network number and name for each network found:
    for (int thisNet = 0; thisNet < numSsid; thisNet++) {
      Serial.print(thisNet);
      Serial.println(") Rede: " + String(WiFi.SSID(thisNet)) + " RSSI: " + String(WiFi.RSSI(thisNet)) );
    }

  }

} Net; 

typedef struct {
  float temperature;
  float humidity;
  float lux;
  int motionSensorState;
} PerceptionLayer;

typedef struct {

  int RT;
  int RB;
  int LT;
  int LB;

} VehicleInfo;

typedef struct struct_message {
  
  bool buttonState;
  int lifeProbability;
  PerceptionLayer perceptionValues;

} struct_message;

// Credenciais Wi-Fi
const String ssidName = "POCO F5";
const String ssidPassword = "miguelFBI22";

//output pins

const int servo1Pin = 18;
const int servo2Pin = 19;

Servo servo1;
Servo servo2;


int lifeProbability = -1;

bool buttonState = true;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

PerceptionLayer perceptionValues;
struct_message dataRecieved;
Net network;

// Capacidade ajustada para um Objeto JSON com 4 chaves
const size_t CAPACITY = 96; 

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
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
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

VehicleInfo vehicleInfo = {0,0,0,0};

void processGamepad(ControllerPtr ctl) {

    vehicleInfo.RT = ctl->throttle();
    vehicleInfo.RB = ctl->r1();
    vehicleInfo.LT = ctl->brake();
    vehicleInfo.LB = ctl->l1();
  
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void rotateMotor(int RT, int RB, int LT, int LB){
  if(LB== 1) LB= -1;
  else LB= 1;

  if(RB== 1) RB= -1;
  else RB= 1;

  int speedServo1 = map(LT * LB, -1023, 1023, 1000, 2000);
  int speedServo2 = map(RT * RB, -1023, 1023, 2000, 1000);

  servo1.writeMicroseconds(speedServo1);
  servo2.writeMicroseconds(speedServo2);

}

void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&dataRecieved, incomingData, sizeof(dataRecieved));
  buttonState       =  dataRecieved.buttonState;
  lifeProbability   =  dataRecieved.lifeProbability;
  perceptionValues  =  dataRecieved.perceptionValues;

}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

    pinMode(servo1Pin, OUTPUT);
    pinMode(servo2Pin, OUTPUT);

    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);

    // 1. Configura o modo de Wi-Fi e Bluetooth (combinado)
    WiFi.mode(WIFI_AP_STA);

    // 2. Conecta à rede
    network.connectToTheNetworkBySSIDName(ssidName, ssidPassword);

    delay(300);

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    // 4. Setup do ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao iniciar ESP-NOW");
        return;
    }
    // Registra callback
    esp_now_register_recv_cb(onDataRecv);
    
    Serial.println("Setup Finalizado");
}

// Arduino loop function. Runs in CPU 1.

int lastLifeProbability = -1;
unsigned long long int lastMillis = 0;

void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    unsigned long long int milis = millis();
    if(milis -lastMillis >= 300) {
    
        if(!buttonState || lifeProbability > 75){
          Serial.println("Maquina Desligada");
          servo1.writeMicroseconds(1500);
          servo2.writeMicroseconds(1500);
          delay(500);
          return;
        }

        Serial.println("RT: " + String(vehicleInfo.RT));
        Serial.println("RB: " + String(vehicleInfo.RB));
        Serial.println("LT: " + String(vehicleInfo.LT));
        Serial.println("LB: " + String(vehicleInfo.LB));

        Serial.println("SENSOR DE TEMPERATURA: "+ String(perceptionValues.temperature));
        Serial.println("SENSOR DE UMIDADE: "+ String(perceptionValues.humidity));
        Serial.println("SENSOR DE PRESENCA: "+ String(perceptionValues.motionSensorState));
        Serial.println("SENSOR DE LUZ: "+ String(perceptionValues.lux));
        Serial.println("Probabilidade de vida: " + String(lifeProbability) +"%");

        rotateMotor(vehicleInfo.RT, vehicleInfo.RB, vehicleInfo.LT, vehicleInfo.LB);
        lastMillis = milis;
    }
    delay(150);
}
