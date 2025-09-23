// ------------------------------------------------------------------------
//   Muster für Windensteuerung mit LoRa für  "Heltec ESP32 LoRa v3"
// ------------------------------------------------------------------------
//  https://www.arduino.cc/reference/en/libraries/heltec_esp32_lora_v3/
//  https://www.arduino.cc/reference/de/
//  https://github.com/jgromes/RadioLib/tree/master/examples/SX126x
//  https://www.arduino.cc/reference/en/libraries/heltec_esp32_lora_v3/
//  https://jgromes.github.io/RadioLib/class_s_x126x.html
// ------------------------------------------------------------------------

//vesc battery number of cells
static int numberOfCells = 20;
static int myMaxPull = 75;  // 0 - 127 [kg], must be scaled with VESC ppm settings

#include <heltec_unofficial.h>
#include "LiPoCheck.h"    //to calculate battery % based on cell Voltage
#include "AiEsp32RotaryEncoder.h"

// Frequency in MHz. Keep the decimal point to designate float.
 #define FREQUENCY           868.0      // for Europe
// #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           125

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR   9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0



// GPIO Pins 0,1,8-14,17,18,21,35,36,37 sind von helltec belegt

//Rotary encoder
HotButton btnup(7, true, LOW);
#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 6
AiEsp32RotaryEncoder rotary = AiEsp32RotaryEncoder(5, 6, 7, -1, 4,false); //PIN A oder CLK=5; PIN B oder DT=6; Taster oder SW (nicht belegt weil schon auf HOTBUTTON=7); Kein VCC Anschluß oder direkt 5V?=-1; RotarySteps=4; Pullup Widerstand?=false
//Using VescUart librarie to read from Vesc (https://github.com/SolidGeek/VescUart/)
#include <VescUart.h>
#define VESC_RX  2    //connect to TX on Vesc rot
#define VESC_TX  3    //connect to RX on Vesc schwarz
VescUart vescUART;

// PWM signal für vesc (Ich nutze die Bibliothek LEDC https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html)
#define PWM_PIN  4 		//Digital PIN
#define PWM_FREQ 50		//Frequenz 50 Hz
#define PWM_RES 10		//10 Bit Auflösung

#define sep ","         //Trennzeichen
String rxdata;
String txdata;

int cnt = 0;
int transmissionState = RADIOLIB_ERR_NONE;  // save transmission states between loops

volatile bool rxFlag = false;
volatile bool txFlag = false; 

int currentId = 1; 		//ID des Senders, kann mit dem Drehgeber verändert werden
int currentState = 0;
int currentPull = 0;
int8_t targetPull = 0;    // empfangen vom Lora Sender

float vescBattery = 0;
float vescTempMotor = 0;
float vescTachometer = 0;
float vescDutyCycle = 0;
unsigned long lastTxLoraMessageMillis = 0;
unsigned long lastRxLoraMessageMillis = 0;
uint32_t  pwmWriteTimeValue = 0;
uint32_t  pulseLength = 0;

const unsigned long RegelungsInterval = 6;  //
unsigned long lastRegelungsTime = 0;   //
const unsigned long loraTimeout = 3000; // 3 Sekunden Timeout

struct AntwortStruktur {
  String status;
  int currentId;
  int currentState;
  int targetPull;
  float neuerWert;
};
AntwortStruktur Antwort;  // Antwort des Senders



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
  radio.begin();
  radio_init();
  Serial.print(F("[SX1262] Initializing ... "));
  radio.clearDio1Action();

  Serial.println(" Ich bin eine Winde");

  radio.setPacketSentAction(txIsr);      //Legt die Interrupt-Service-Routine fest, die aufgerufen wird, wenn ein Paket gesendet wurde.
  radio.setPacketReceivedAction(rxIsr);  // Legt die Interrupt-Service-Routine fest, die aufgerufen wird, wenn ein Paket empfangen wurde.
  StartReceive();                        //Start Lesen von LoRa-Paketen

  // Dislplay Einstellungen
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.println("Windensteuerung");
  delay(1000);
  display.setFont(ArialMT_Plain_10);

  // Drehimpulsgeber
  rotary.begin();
  rotary.setup([]{rotary.readEncoder_ISR();});
  rotary.setBoundaries(1, 5, false); //Sender ID von 1 bis 5
  rotary.setAcceleration(0);           // keine beschleunigte Werteänderung bei schnellem Drehen
  rotary.setEncoderValue(1);          //Grundwert einstellen auf ID 1

  // Setup UART port für Vesc Kommunikation
  Serial1.begin(115200, SERIAL_8N1, VESC_RX, VESC_TX);
  vescUART.setSerialPort(&Serial1);

  //PWM Pins
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
}

//-------------------------------------------------------------------
void loop()
//-------------------------------------------------------------------
{
  //---------------------------------------------------------------------------------------------------------------------
  //radio.setSyncWord(0x..); Das Sync-Wort ist im Grunde die Netzwerk-ID des Funkgeräts. Funkgeräte mit unterschiedlichen
  //Sync Words können die Übertragungen der jeweils anderen nicht empfangen. Dies ist eine Möglichkeit
  //Funkgeräte herausfiltern, die Sie ignorieren möchten, ohne ein Adressierungsschema zu erstellen.
  //---------------------------------------------------------------------------------------------------------------------
  if (btnup.pressedNow()) {
    currentId = (rotary.readEncoder());
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);  //10, 16, 24
    display.drawString(0, 5, String("Sender ID:"));
    display.setFont(ArialMT_Plain_24);  //10, 16, 24
    display.drawString(0, 30, String(currentId) );
    display.display();
  }
  if (currentId == 1){
    radio.setSyncWord(0x21);  //LoRa sync word to be set.
  } else if (currentId == 2) {
    radio.setSyncWord(0x32);  //LoRa sync word to be set.
  } else if (currentId == 3) {
    radio.setSyncWord(0x43);  //LoRa sync word to be set.
  } else if (currentId == 4) {
    radio.setSyncWord(0x54);  //LoRa sync word to be set.
  } else if (currentId == 5) {
    radio.setSyncWord(0x65);  //LoRa sync word to be set.
  }

  
  //Kommastellen abschneiden
  float RSSI=radio.getRSSI(); int rssi=(int)RSSI;
  float SNR=radio.getSNR(); int snr=(int)SNR;
  int vescbattery=(int)vescBattery;

  /*-------------------------------------------------------------------
  // Windensteuerung
  //-------------------------------------------------------------------
  // Regelung PID etc. periodisch aufrufen
  if (millis() - lastRegelungsTime > RegelungsInterval) {
    lastRegelungsTime = millis();
    Windensteuerung();
  }
*/
  //-------------------------------------------------------------------
  // Display
  //-------------------------------------------------------------------
  if (!btnup.pressedNow()) {   //damit das Display nicht flackert wenn der PROG Butten gehalten wird
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      display.drawString(0, 0, currentId + String(" | ") + rssi + "dBm | " + snr + "dBm | " + vescbattery + "%");
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      display.drawString(0, 11, String(Antwort.currentState) + " (" + String(Antwort.targetPull) + "kg)");    
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      display.drawString(0, 36, String("L: ") + (vescTachometer/100) + "m");
      //display.drawString(0, 48, String("Last TX / RX: ") + lastTxLoraMessageMillis/100 + " / " + lastRxLoraMessageMillis/100);
      display.display();
    
  }

  //-------------------------------------------------------------------
  // Windenkommunikation
  //-------------------------------------------------------------------
  if (rxFlag) {          // RX Flag wird gesetzt wenn ein Paktet VOLLSTÄNDIG empfangen wurde
    lastRxLoraMessageMillis = millis();
    Serial.println("----------------------------------------");
    rxFlag = false;
    rxdata = EmpfangeNachricht();
    //String zerlegen ween mehrer Float Werte in der Nachricht sind
    Antwort = ParseString(rxdata);
        Serial.print("ID: ");
        Serial.print(Antwort.currentId);
        Serial.print(" - Status: ");
        Serial.println(Antwort.currentState);
    Vesc_Daten();
    //      Serial.println(vescTachometer);
    //      Serial.println(vescTempMotor);
    //      Serial.println(vescBattery);
    //      Serial.println(vescDutyCycle); //drehzahl


    //-------------------------------------------------------------------
    // sende Nachricht an Sender zurück
    //-------------------------------------------------------------------
    SendeNachricht(String(currentPull) + String(sep) + String(vescBattery, 0) + String(sep) + String(vescTempMotor, 0) + String(sep) + String(vescTachometer/100, 0));
  }
  
  // TX Flag wird gesetzt wenn ein Paktet VOLLSTÄNDIG gesendet wurde
  if (txFlag) {  
    lastTxLoraMessageMillis = millis();
    txFlag = false;
    StartReceive();  //starte Interrupt-gesteuerte Empfangsmethode
  }


  //-------------------------------------------------------------------
  // Übergebe Zugkraft
  //-------------------------------------------------------------------
  targetPull = Antwort.targetPull;
  if (targetPull < 0) {
    currentPull = 0;
  } else {
    currentPull = targetPull;
    currentPull = constrain(currentPull, 0, 110); // Hält den Wert im Bereich
  }
  // bei Fehler wenn kein Target Pull empfangen wird:
  if (millis() - lastRxLoraMessageMillis > loraTimeout) {
    // Timeout erreicht
    Serial.println("Verbindung verloren!");
    currentPull = 0;
  }


  //------------------------------------------------------------------------
  // PWM Berechnung und Ausgabe 
  //------------------------------------------------------------------------

  // Mein Motor QS205 5000W 5T hat eine lineare Kennlinie. Bei einer Pulsbreite von 150 geht es los mit ca. 1kg
  // Bei einer Pulsbreite von 900 habe ich 70kg Zugkraft. Y ist Zugkraft und X Pulsbreite
  // f(x)=mX+n ---> m=(Y2-Y1)/(X2-X1) ---> m=(70-1)/(900-150) m rund 0,095
  // Nullstelle bei Pulsbreite 140: 0=mX+n ---> 0=0,095*140+n ---> n=-13
  //  Zugkraft(Y)=0.095*Pulsbreite(X)-13 | +13 --->  0.095*X=Y+13	| :0.095 ---> 
  // Pulsbreite(X)=10.526*Zugkraft(Y)+136.842
  // |
  // |                        .
  // |                     .
  // |                  .
  // |               .
  // |            .
  // |         .
  // |      .
  // |_________________________________

  // lese Tachometerwert:
  Vesc_Daten();

  // Überprüfe den Tachometerwert, um den Betriebsmodus festzulegen
  // TODO: Bremse einbauen für kürzeres Seil ca. 1500
  if (vescTachometer <200 ) {
  // Zustand 1: Normaler Betrieb (mehr als X Meter Seil abgewickelt)

  
  if (button.pressedNow()) {
    // Prüfe, ob der Button gedrückt wird, um die normale Steuerung zu überbrücken
    Serial.println("Manuelles Einholen: Button gedrückt (vor 60m Grenze)");
    ledcWrite(PWM_PIN, 200); // Manuelles Einholen mit PWM 200

  } else {
    // Wenn der Button nicht gedrückt ist, nutze die normale Steuerung
    // Serial.println("Normale Steuerung (vor 60m Grenze)");
    pwmWriteTimeValue = 10.526 * currentPull + 137;
    pwmWriteTimeValue = constrain(pwmWriteTimeValue, 0, 900);
    ledcWrite(PWM_PIN, pwmWriteTimeValue);
    }
  }
  else {
  // Zustand 2: Sicherheit und manuelles Einholen (weniger als X Meter Seil abgewickelt oder bereits mehr eingezogen)

  // Wenn der Tachometerwert -6000 oder größer ist, hat der Button die Kontrolle
  if (button.pressedNow()) {
    Serial.println("Manuelles Einholen der letzten 60m: Button gedrückt");
    ledcWrite(PWM_PIN, 200); // Manuelles Einholen mit PWM 200

  } else {
    // Wenn der Button NICHT gedrückt ist, schalte die Winde ab
    Serial.println("Sicherheitsabschaltung: < 60m Seil und Button nicht gedrückt");
    ledcWrite(PWM_PIN, 1);
    } 
  }
  Serial.println(pwmWriteTimeValue);

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
    //  Paket wurde richtig empfangen
    Serial.printf("Paket empfangen\n");
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
  Serial.printf("Frequency: %.2f MHz\n", FREQUENCY);
  radio.setFrequency(FREQUENCY);
  Serial.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  radio.setBandwidth(BANDWIDTH);
  Serial.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  radio.setSpreadingFactor(SPREADING_FACTOR);
  Serial.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  radio.setOutputPower(TRANSMIT_POWER);
}

//-------------------------------------------------------------------
void Vesc_Daten()
//-------------------------------------------------------------------
{   if (vescUART.getVescValues()) {
      vescBattery = CapCheckPerc(vescUART.data.inpVoltage, numberOfCells);    // vesc battery in %
      vescTempMotor = vescUART.data.tempMotor;                                // motor temp in C     
      vescTachometer = vescUART.data.tachometer;
      vescDutyCycle = vescUART.data.dutyCycleNow;    
    } 
    else {
//      Serial.println("Failed to get data from VESC!");
    }
}

/*------------------------------------------------------------------------
void Windensteuerung()
//------------------------------------------------------------------------
{  // hier verbrate ich 5 msec um zu prüfen ob trotzdem alles funkt
  do {
      //Serial.println("da brat ich mir nen Storch");
  } while (millis() - lastRegelungsTime > 3);
}
*/
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
        Antwort.currentId = Record.substring(i, j).toFloat();
        break;
      case 1:  //
        Antwort.currentState = Record.substring(i, j).toFloat();
        break;
      case 2:  //
        Antwort.targetPull = Record.substring(i, j).toFloat();
        break;
      case 3:  //
        Antwort.neuerWert = Record.substring(i, j).toFloat();
        break;
    }
    i = j + 1;
    counter++;
  } while (j <= Record.length());
  return Antwort;
}


