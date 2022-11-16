#include "BluetoothSerial.h"
#include "ELMduino.h"


BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial

#define REMOVE_BONDED_DEVICES 1

ELM327 myELM327;


uint32_t rpm = 0;
uint32_t kph = 0;


//>>>>>>>>>>>Found a new device asynchronously: Name: OBDII, Address: 01:23:45:67:89:ba, cod: 7936, rssi: -85
uint8_t address[6]  = {0x01, 0x23, 0x45, 0x67, 0x89, 0xba};






#define BT_DISCOVER_TIME  10000
esp_spp_sec_t sec_mask=ESP_SPP_SEC_NONE; // or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE to request pincode confirmation
esp_spp_role_t role=ESP_SPP_ROLE_SLAVE; // or ESP_SPP_ROLE_MASTER

// std::map<BTAddress, BTAdvertisedDeviceSet> btDeviceList;

void setup_bt() {
  Serial.begin(115200);
  if(! SerialBT.begin("ESP32test", true) ) {
    Serial.println("========== serialBT failed!");
    abort();
  }
  // SerialBT.setPin("1234"); // doesn't seem to change anything
  // SerialBT.enableSSP(); // doesn't seem to change anything


  Serial.println("Starting discoverAsync...");
  BTScanResults* btDeviceList = SerialBT.getScanResults();  // maybe accessing from different threads!
  if (SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) {
      // BTAdvertisedDeviceSet*set = reinterpret_cast<BTAdvertisedDeviceSet*>(pDevice);
      // btDeviceList[pDevice->getAddress()] = * set;
      Serial.printf(">>>>>>>>>>>Found a new device asynchronously: %s\n", pDevice->toString().c_str());
    } )
    ) {
    delay(BT_DISCOVER_TIME);
    Serial.print("Stopping discoverAsync... ");
    SerialBT.discoverAsyncStop();
    Serial.println("discoverAsync stopped");
    delay(5000);
    if(btDeviceList->getCount() > 0) {
      BTAddress addr;
      int channel=0;
      Serial.println("Found devices:");
      for (int i=0; i < btDeviceList->getCount(); i++) {
        BTAdvertisedDevice *device=btDeviceList->getDevice(i);
        Serial.printf(" ----- %s  %s %d\n", device->getAddress().toString().c_str(), device->getName().c_str(), device->getRSSI());
        std::map<int,std::string> channels=SerialBT.getChannels(device->getAddress());
        Serial.printf("scanned for services, found %d\n", channels.size());
        for(auto const &entry : channels) {
          Serial.printf("     channel %d (%s)\n", entry.first, entry.second.c_str());
        }
        if(channels.size() > 0) {
          addr = device->getAddress();
          channel=channels.begin()->first;
        }
      }
      if(addr) {
        Serial.printf("connecting to %s - %d\n", addr.toString().c_str(), channel);
        SerialBT.connect(addr, channel, sec_mask, role);
      }
    } else {
      Serial.println("Didn't find any devices");
    }
  } else {
    Serial.println("Error on discoverAsync f.e. not workin after a \"connect\"");
  }
}

void setup()
{
#if LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  // DEBUG_PORT.begin(115200);
  // ELM_PORT.begin("ArduHUD", true);
  // SerialBT.setPin("1234");
  
  // // if (!ELM_PORT.connect("OBDII"))
  // if (!ELM_PORT.connect("address"))
  // {
  //   DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
  //   while(1);
  // }


  setup_bt();

  if (!myELM327.begin(ELM_PORT, false, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
    while (1);
  }

  Serial.println("Connected to ELM327");
}


void loop()
{
  // float tempRPM = myELM327.rpm();

  // if (myELM327.nb_rx_state == ELM_SUCCESS)
  // {
  //   rpm = (uint32_t)tempRPM;
  //   Serial.print("RPM: "); Serial.println(rpm);
  // }
  // else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
  // {
  //   myELM327.printError();
  // }

  float tempKPH = myELM327.kph();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    kph = tempKPH;
    Serial.print("KPH: "); Serial.println(kph);
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
  {
    myELM327.printError();
  }
}
