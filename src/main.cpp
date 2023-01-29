#include <Arduino.h>

/**
 * ----------------------------------------------------------------------------
 * This is a MFRC522 library example; see https://github.com/miguelbalboa/rfid
 * for further details and other examples.
 * 
 * NOTE: The library file MFRC522.h has a lot of useful info. Please read it.
 * 
 * Released into the public domain.
 * ----------------------------------------------------------------------------
 * Minimal example how to use the interrupts to read the UID of a MIFARE Classic PICC
 * (= card/tag).
 * 
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        3                10
 * IRQ         ?            ?             ?         ?          2                10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 * More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
 * 
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define RST_PIN         15           // Configurable, see typical pin layout above
#define SS_PIN          5           // Configurable, see typical pin layout above
#define IRQ_PIN         4           // Configurable, depends on hardware
#define INTERRUPTOR     2           //PIN de salida, indica que se ejecuta una interrupcion del tablero de luces
#define LATCH           14          //LATCH para el shifter
#define CLK             13          //CLK para el shifter
#define DATA            12          //DATA serial para el Shifter
#define NMiembrosGDSC   8           //NUMERO DE MIEMBROS DEL CORE TEAM GDSC UNI
#define RELE_CERROJO    26
#define SegundosPuertaAbierta 5



MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

MFRC522::MIFARE_Key key;

LiquidCrystal_I2C	lcd(0x27,16,2); //Create LCD instance. 0x27 es la direccion de memoria, si no funciona probar con 0x3F. 16 y 2 son las cols y fils respectivamnte

//Variables para el manejo del RFID.
volatile bool bNewInt = false;
byte regVal = 0x7F;
//Almacenamiento de la cadena ID de las tarjetas
String BufferID;


//Variables para el manejo del Juego de luces.
bool arregloAsistencia [NMiembrosGDSC] = {0,0,0,0,0,0,0,0};//Tabla de verdadero y falso de los miembros

String Identifier [NMiembrosGDSC] = {"4ae7d180","93bcc01d","e76e8d4a","1358c31d","","","",""}; //ID de los miembros del GDSC UNI
String CoreTeamNames [NMiembrosGDSC] = {"Renato","Gengis","Eduardo","Jossef","Abigail","José","Adrian","Keru xd"}; //ID de los miembros del GDSC UNI
int i;

//funcion para la comparación de los ID con el ingresado al RFID
int compareID();

//funciones para el RFID.
void activateRec(MFRC522 mfrc522);
void clearInt(MFRC522 mfrc522);
void IRAM_ATTR readCard();
void dump_byte_array(byte *buffer, byte bufferSize);

//funcion para el juego de luces.
void funcionDeInterrupcion();

//funcion para abrir y cerrar puerta, con temporizador
void IRAM_ATTR onTimer(); 
hw_timer_t *timer = NULL; //apuntador/puntero
volatile bool trigger=false;
/**
 * Initialize.
 */
void setup() {
  Serial.begin(9600); // Initialize serial communications with the PC
  while (!Serial);      // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  lcd.begin();                      // initialize the lcd 
  lcd.backlight();                  //luz trasera

  SPI.begin();          // Init SPI bus


  mfrc522.PCD_Init(); // Init MFRC522 card


  /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
  Serial.print(F("Ver: 0x"));
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println(readReg, HEX); 

  /* setup the IRQ pin*/
  pinMode(IRQ_PIN, INPUT_PULLUP);

  /*
   * Allow the ... irq to be propagated to the IRQ pin
   * For test purposes propagate the IdleIrq and loAlert
   */
  regVal = 0xA0; //rx irq
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);

  bNewInt = false; //interrupt flag

  /*Activate the interrupt*/
  attachInterrupt(IRQ_PIN, readCard, FALLING);

  //do { //clear a spourious interrupt at start
  //  ;
  //} while (!bNewInt);
  bNewInt = false;

  pinMode(INTERRUPTOR, OUTPUT);//Inicializacion del interruptor
  pinMode(LATCH, OUTPUT); //Inicializacion del latch
  pinMode(CLK, OUTPUT);//inicialización del clk
  pinMode(DATA, OUTPUT);//inicualización de data

  pinMode(RELE_CERROJO,OUTPUT);
  digitalWrite(RELE_CERROJO,LOW);

  //Condiciones iniciales.
  digitalWrite(LATCH,LOW);
  digitalWrite(CLK, LOW);
  funcionDeInterrupcion();
  
  i=0;

  
  timer = timerBegin(0, 80, true); //inicializacion del timer
  timerAttachInterrupt(timer, &onTimer, true); 
  timerAlarmWrite(timer, SegundosPuertaAbierta*1000000, true);
  //timerAlarmDisable(timer);
  



  Serial.println(F("End setup"));
  
  lcd.print("Bienvenido al");
  lcd.setCursor(0,1);
  lcd.print("GDSC UNI");
}

/**
 * Main loop.
 */
void loop() {
  if(trigger){
    //timerStop(timer);
    timerAlarmDisable(timer);
    lcd.clear();
    lcd.print("Bienvenido al");
    lcd.setCursor(0,1);
    lcd.print("GDSC UNI");
    trigger=false;
    digitalWrite(RELE_CERROJO,LOW);
    digitalWrite(INTERRUPTOR,LOW);
  }
  if (bNewInt) { //new read interrupt
    digitalWrite(INTERRUPTOR,HIGH);
    Serial.print(F("Interrupt. "));
    mfrc522.PICC_ReadCardSerial(); //read the tag data
    // Show some details of the PICC (that is: the tag/card)
    Serial.print(F("Card UID:"));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    
    clearInt(mfrc522);
    mfrc522.PICC_HaltA();
    bNewInt = false;

    i=compareID();
    if(i==NMiembrosGDSC || i<0 || i>NMiembrosGDSC){
      Serial.println("\nNo permite acceso");
      lcd.clear();
      lcd.print(" Acceso Denegado");
    }
    else{
      arregloAsistencia[i]=!arregloAsistencia[i];
      lcd.clear();
      lcd.setCursor(0,0);
      arregloAsistencia[i]==true ? lcd.print("Bienvenido/a ") : lcd.print("Hasta pronto");
      lcd.setCursor(0,1);
      lcd.print(CoreTeamNames[i]);
      digitalWrite(RELE_CERROJO,HIGH);
    }
    
    funcionDeInterrupcion();
    delay(200);
    

    BufferID = "";
    timerRestart(timer);
    timerAlarmEnable(timer);
  }

  // The receiving block needs regular retriggering (tell the tag it should transmit??)
  // (mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg,mfrc522.PICC_CMD_REQA);)
  activateRec(mfrc522);
  delay(100);
} //loop()

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
    BufferID = BufferID + String(buffer[i], HEX);
  }
}

int compareID(){
  bool door=true;
  for(i=0;(i<NMiembrosGDSC)&&(door);i++){
    if(BufferID.compareTo(Identifier[i]) == 0){
      door = false;
    }
  }
  if (door){
    return NMiembrosGDSC;
  }
  return --i;
}

/**
 * MFRC522 interrupt serving routine
 */
void IRAM_ATTR readCard() {
  bNewInt = true;
}

void funcionDeInterrupcion()
{
  
  for(int j=NMiembrosGDSC;j>=0;j--){
    digitalWrite(DATA, arregloAsistencia[j]);
    digitalWrite(CLK,HIGH);
    digitalWrite(CLK,LOW);
  }
  Serial.println();
  digitalWrite(LATCH, HIGH);
  digitalWrite(LATCH, LOW);
  //timerAlarmEnable(timer);
  
}


/*
 * The function sending to the MFRC522 the needed commands to activate the reception
 */
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

/*
 * The function to clear the pending interrupt bits after interrupt serving routine
 */
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}

void IRAM_ATTR onTimer() {
  trigger = true;
}