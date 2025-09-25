// ------------------------------------------------------------------------
//   Muster für Windensteuerung mit LoRa für  "Heltec ESP32 LoRa v3.2"
// ------------------------------------------------------------------------
//  heltec_unofficial.h - https://github.com/ropg/heltec_esp32_lora_v3
//  Idee: https://github.com/robertzach/ewinch_remote_controller
//  Dank an RolfK https://www.rc-network.de/threads/hochstartwinde-funkgesteuert.641930/page-26#post-12857593
// ------------------------------------------------------------------------
static int myID = 1;    // set to your desired transmitter id!!! [unique number from 1 - 15]
static int myMaxPull = 75;  // 0 - 127 [kg], must be scaled with VESC ppm settings

#include <heltec_unofficial.h>
#include "AiEsp32RotaryEncoder.h"

// Frequency in MHz. Keep the decimal point to designate float.
#define FREQUENCY           868.0       // for Europe
// #define FREQUENCY           915.0       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           125.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0

HotButton btnup(7, true, LOW);
#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 6
AiEsp32RotaryEncoder rotary = AiEsp32RotaryEncoder(5, 6, 4, -1, 4,false); //PIN A oder CLK=5; PIN B oder DT=6; Taster oder SW (nicht belegt weil schon auf HOTBUTTON)=4; Kein VCC Anschluß oder direkt 5V?=-1; RotarySteps=4; Pullup Widerstand?=false

#define sep ","            //Trennzeichen

String rxdata;
String txdata;

int cnt = 0;
int transmissionState = RADIOLIB_ERR_NONE;  // save transmission states between loops

volatile int count;
volatile bool rxFlag = false;
volatile bool txFlag = false;
unsigned long letzteAktion = 0;        //
unsigned long Pause = 1000;            //
unsigned long RegelungsInterval = 6;  //
unsigned long lastRegelungsTime = 0;   //
unsigned long SendTimeOut = 3000;       //
bool AntwortErhalten = true;
static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;   // pull value range from -127 to 127
int currentPull = 0;          // current active pull on vesc
bool stateChanged = false;
int currentState = 0;   // 0 = no pull/no brake, 1 = default pull (~7kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
int defaultPull = 7;  //in kg
int prePullScale = 20;      //in %
int takeOffPullScale = 55;  //in %
int fullPullScale = 80;     //in %
int strongPullScale = 100;  //in %
int rewinchPull = 10;
//uint8_t vescBattery = 0;
//uint8_t vescTempMotor = 0;
//uint8_t vescTachometer = 0;


struct AntwortStruktur {
  int currentPull;
  int vescBattery;
  int vescTempMotor;
  int vescTachometer;
  float neuerWert;
};
AntwortStruktur Antwort;  // Antwort der Winde

//-------------------------------------------------------------------
void setup()
//-------------------------------------------------------------------
{
  gpio_install_isr_service(0);
  heltec_setup();
  Serial.begin(115200);
  heltec_ve(true);
  Serial.println("Radio init");

  //------------------------------------------------------------
  // initialize SX1262 with settings
  //------------------------------------------------------------

  Serial.print(F("[SX1262] Initializing ... "));
  radio.begin ();
  radio_init();
  radio.clearDio1Action();
  //---------------------------------------------------------------------------------------------------------------------
  //radio.setSyncWord(0x..); Das Sync-Wort ist im Grunde die Netzwerk-ID des Funkgeräts. Funkgeräte mit unterschiedlichen
  //Sync Words können die Übertragungen der jeweils anderen nicht empfangen. Dies ist eine Möglichkeit
  //Funkgeräte herausfiltern, die Sie ignorieren möchten, ohne ein Adressierungsschema zu erstellen.
  //---------------------------------------------------------------------------------------------------------------------
  radio.setSyncWord(0x21);  // ID1=0x21, ID2=0x32, ID3=0c43, ID4=0x54, ID5=65

  Serial.println(" Ich bin ein Sender");

  radio.setPacketSentAction(txIsr);      //Legt die Interrupt-Service-Routine fest, die aufgerufen wird, wenn ein Paket gesendet wurde.
  radio.setPacketReceivedAction(rxIsr);  // Legt die Interrupt-Service-Routine fest, die aufgerufen wird, wenn ein Paket empfangen wurde.
  StartReceive();                        //Start Lesen von LoRa-Paketen

  // Dislplay Einstellungen
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.println("Windensteuerung");
  delay(1000);
  display.setFont(ArialMT_Plain_10);

  //Drehimpulsgeber
  rotary.begin();
  rotary.setup([]{rotary.readEncoder_ISR();});
  rotary.setBoundaries(60, 110, false); //Minimale und maximale einstellbares Gewicht
  rotary.setAcceleration(0);           // keine beschleunigte Werteänderung bei schnellem Drehen
  rotary.setEncoderValue(75);          //Grundwert einstellen auf 75 kg

}
//-------------------------------------------------------------------
void loop()
//-------------------------------------------------------------------
{
  //Kommastellen abschneiden
  float RSSI=radio.getRSSI(); int rssi=(int)RSSI;
  float SNR=radio.getSNR(); int snr=(int)SNR;

  //-------------------------------------------------------------------
  // Display
  //-------------------------------------------------------------------
  if (!button.pressedNow()) {  //damit das Display nicht flackert wenn der PROG Butten gehalten wird
    if (currentState > -1 && currentState < 6){
      loopStep++;
      if (loopStep % 100 == 0) {
        toogleSlow = !toogleSlow;
      }
      if (loopStep % 10 == 0) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_10);  //10, 16, 24
        if (toogleSlow) {
          display.drawString(0, 0, myID + String(" | ") + String(Antwort.vescBattery) + "% | "  + String(Antwort.vescTempMotor) + "C");
        } else {
          display.drawString(0, 0, myID + String(" | ") + heltec_battery_percent() + "% | " + rssi + "dBm |" + snr + "dBm");        

        }
        display.setFont(ArialMT_Plain_24);  //10, 16, 24
        display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + String(Antwort.currentPull) + String("kg)"));
        //display.drawString(0, 36, String(Antwort.vescTempMotor) + "C| " + String(Antwort.vescBattery) + "%" );
        display.drawString(0, 36, String(Antwort.vescTachometer) + "m | " + String(Antwort.vescTempMotor) + "°" );
        display.display();
      }
    }
    if (currentState ==9){
      targetPull = rewinchPull;
      display.clear();
        
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 0, String(currentState) + String(" (") + targetPull + "/" + String(Antwort.currentPull) + String("kg)"));
        display.setFont(ArialMT_Plain_24);  //10, 16, 24
        display.drawString(6, 18, String("Rewinch"));
        display.drawString(6, 40, String(Antwort.vescTachometer) + "m");
        display.display();

    }
  }
 if (millis() - letzteAktion > Pause) {
    letzteAktion = millis();
    AntwortErhalten = false;

    // SendeNachricht(StartWinde + String(sep) + String(SollSeilspannung, 0));
    SendeNachricht(String(myID) + String(sep) + String(currentState) + String(sep) + String(targetPull));
  }

  //-------------------------------------------------------------------
  if (txFlag) {  // Paket wurde vollständig gesendet

    txFlag = false;
    StartReceive();  //starte Interrupt-gesteuerte Empfangsmethode
  }

  //-------------------------------------------------------------------
  // Fehlerbehandlung
  //-------------------------------------------------------------------

  if ((millis() - letzteAktion > SendTimeOut) &&!AntwortErhalten) {
    Serial.printf("Winde antwortet nicht !!!! \n");
    targetPull = 0;
    AntwortErhalten = true;  // damit die Meldung nur einmal kommt
  } 



  //-------------------------------------------------------------------
  // Antworten empfangen
  //-------------------------------------------------------------------
  if (rxFlag) { 
    heltec_led(100); // Paket wurde empfangen
    rxFlag = false;
    rxdata = EmpfangeNachricht();
      //String zerlegen ween mehrer Float Werte in der Nachricht sind
      Antwort = ParseString(rxdata);
      Serial.print("Spannung : ");
      Serial.print(Antwort.vescBattery);
      Serial.print(" Temp : ");
      Serial.println(Antwort.vescTempMotor);
      AntwortErhalten = true;
    } 
    else {
      heltec_led(0);
    }

  delay(10);


  btnup.update();
  if (btnup.isSingleClick()) {
  Serial.println("Einfach hoch\n");
  if ( currentState < 5) {
        currentState = currentState + 1;
        stateChanged = true;
        if (currentState == 0 ) {
            currentState = currentState + 1;
            stateChanged = true;
        }
      }
    }

  if (btnup.isDoubleClick()) {
  //Serial.println("Doppelt runter\n");
  if (currentState > 1) {
    currentState = currentState - 1;
    stateChanged = true;
    }
  }

  if (btnup.pressedFor(1500)) {
  //  Serial.println("Lange auf 1 \n");
    if ( currentState > 0) {
        currentState = 9;
        stateChanged = true;
    } 
  }
      
  if (btnup.isTripleClick()) {
  //Serial.println("dreifach auf 0\n");
    currentState = 0;
    stateChanged = true;
  }

  // state machine
  // -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
  switch(currentState) {
    case 0:
      targetPull = 0; // -> neutral, no pull / no brake
      break;
    case 1: 
      targetPull = defaultPull;   //independent of max pull
      break;
    case 2: 
      targetPull = myMaxPull * prePullScale / 100;
      break;
    case 3:
      targetPull = myMaxPull * takeOffPullScale / 100;
      break;
    case 4:
      targetPull = myMaxPull * fullPullScale / 100;
      break;
    case 5:
      targetPull = myMaxPull * strongPullScale / 100;
      break;
    case 9:
      targetPull = rewinchPull / 100;
      break;
    default: 
      targetPull = 0;
      Serial.println("no valid state");
      break;
  }
  delay(10);

  if (button.pressedNow()) {

    myMaxPull = (rotary.readEncoder());
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);  //10, 16, 24
      display.drawString(0, 5, String("Pilotengewicht:"));
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      display.drawString(0, 30, String(myMaxPull) + "kg" );
      display.display();
  }


}

//------------------------------------------------------------------------
// Interrupt Service Routinen
//------------------------------------------------------------------------
void txIsr() {  // txIsr  Interrupte wird ausgelöst wenn Paktet VOLLSTÄNDIG gesendet wurde
  txFlag = true;
}
void rxIsr() {  // // rxIsr  Interrupte wird ausgelöst wenn Paktet VOLLSTÄNDIG empfangen wurde
  // we got a packet, set the flag
  rxFlag = true;
}

//------------------------------------------------------------------------
void SendeNachricht(String Nachricht)
//------------------------------------------------------------------------
{
  radio.clearPacketReceivedAction();  //Löscht die Interrupt-Service-Routine, die aufgerufen wird, wenn ein Paket empfangen wurde.
  radio.setPacketSentAction(txIsr);

  transmissionState = radio.startTransmit(Nachricht);
  if (_radiolib_status == RADIOLIB_ERR_NONE) {
    Serial.printf("TX  [%s]\n", (Nachricht.c_str()));

  } else {
    Serial.printf("Error (%i)\n", _radiolib_status);
  }
}
//------------------------------------------------------------------------
String EmpfangeNachricht()
//------------------------------------------------------------------------
{
  cnt++;
  // Serial.println(cnt);
  String Nachricht;
  int state = radio.readData(Nachricht);
  if (state == RADIOLIB_ERR_NONE) {
    //  packet was successfully received
    Serial.printf("Paket empfangen !\n");
    Serial.printf("RX(%s)\n", Nachricht.c_str());
  } else {
    Serial.printf("Error (%i)\n", _radiolib_status);
  }
  return Nachricht;
}
//------------------------------------------------------------------------
void StartReceive()
//------------------------------------------------------------------------
{
  radio.clearPacketSentAction();
  radio.setPacketReceivedAction(rxIsr);

  // starte Interrupt-gesteuerte Empfangsmethode mit Standardparametern. Implementiert für die Kompatibilität mit PhysicalLayer.
  int state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    //   Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}
//------------------------------------------------------------------------
void radio_init()
//------------------------------------------------------------------------
{
  // die folgeden Einstellungen müssen bei Sender und Empfänger gleich sein
  // Wenn man keine Angaben macht, werden default Einstellungen übernommen

  Serial.printf("Frequency: %.2f MHz\n", FREQUENCY);
  radio.setFrequency(FREQUENCY);
  Serial.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  radio.setBandwidth(BANDWIDTH);
  Serial.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  radio.setSpreadingFactor(SPREADING_FACTOR);
  Serial.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  radio.setOutputPower(TRANSMIT_POWER);
}
//------------------------------------------------------------------------
void Windensteuerung()
//------------------------------------------------------------------------
{  // hier verbrate ich 2 msec um zu prüfen ob trotzdem alles funkt
  do {
    //  Serial.println("da brat ich mir nen Storch");
  } while (millis() - lastRegelungsTime > 2);
}

// --------------------------------------------------------------
AntwortStruktur ParseString(String Record)
// --------------------------------------------------------------
{
  // Serial.println(Record);

  int i = 0;
  int j = 0;
  int counter = 0;
  do {
    j = Record.indexOf(",", i);
    String tmp = (Record.substring(i, j));
    // int size = (tmp.length());
    switch (counter) {
      case 0:  // is a character array
        Antwort.currentPull = Record.substring(i, j).toFloat();
        break;
      case 1:  //
        Antwort.vescBattery = Record.substring(i, j).toFloat();
        break;
      case 2:  //
        Antwort.vescTempMotor = Record.substring(i, j).toFloat();
        break;
      case 3:  //
        Antwort.vescTachometer = Record.substring(i, j).toFloat();
        break;
        case 4:  //
        Antwort.neuerWert = Record.substring(i, j).toFloat();
        break;
    }
    i = j + 1;
    counter++;
  } while (j <= Record.length());
  return Antwort;
}