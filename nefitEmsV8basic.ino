/*
Uitlezen van NEFIT/BUDERUS EMS-bus 

Gebaseerd op informatie van: http://www.mikrocontroller.net/topic/141831#new
Gebruikt analoog circuit van deze wiki: http://wiki.neo-soft.org/index.php/Heizungsschnittstelle#RC30.2F35.2FEMS-Bus

Het analoge cicruit is aangepast met optocoupler verbindingen naar de ATMEGA, en stroombeveiliging aan de ingang.
Dit maakt verder niets uit voor de software...


Eerste versie: 16 april 2013 Jan van der Meer
Aangepast voor werken op Uno met SoftwareSerialNode library 25 mei 2013 Jan van der Meer, dit is nodig omdat
het Nefit protocol een character-frame-error gebruikt als data-einde-vlag

Versie 6d: 20140105 weer teruggezet naar Mega met gebruik hardware UARTs.
Dit past beter in Nodo concept. Lijkt er op dat het niet mogelijk is om EEn serial port te gebruiken.
Het is nodig om de UART op 9 data bits te zetten, om de zero-stop-bit te kunnen sturen.
Krijg de ontvangst dan niet goed, beetje vreemd zou moeten kunnen.
De NODO gebruikt serial 2 en 3 niet dus eigenlijk geen probleem.

Versie 8basic: 20140202 nieuwe structuur door gebruik te maken van plugin-voor-nodo code
               Deze code loopt sterk parallel met de Plugin code voor de Nodo.
               Een aantal Nodo zaken die niet worden gebruikt zitten er dus nog in (bv. regToWrite en regValue)
               Deze code werkt op een Arduino-Ethernet(UNO), de std serial port op PD0 en PD1 wordt
               gebruikt als interface voor de Nefit-hardware.

debuggen  20140207: correctie offset retourtemp, aanpassen vartype 0 fouten bij bitselectie
aangepast 20140328: acknowledge er weer uitgehaald, lijkt niet nodig

 
 */
 
//#define SKETCH_PATH E:\Users\jan\My Documents\_werk\domotica\nefit\nefitEmsV8basic
#define SKETCH_PATH C:\Users\yucatan\Desktop\nefitEmsV8basic\nefitEmsV8basic
#define NEFIT_REG_MAX	17
 

 
#include <NefitSerial.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <stdint.h>


/*
* omzetten van register naar nefit bus message, en info formaat antwoord
* tabelformaat: <zender>,<frametype>,<data-offset>,<vartype>     // data-offset begint bij byte 4
* vartype bit 3-0 	= 0 boolean (bitnr in bit 6-4)
*			= 1 byte
*			= 2 ascii
*			= 3 int
*			= 4 byte x 2 (voor uitlezen waarde door twee delen)
*			= 5 int x 10 (voor uitlezen door 10 delen)
*                       = 6 byte x 10 (voor uitlezen door 10 delen)
*
*/

PROGMEM const unsigned char regNefitCoding[]={

0x08,0x18,0x01,0x05,					//#1   0 uitgaande cv water x 10
0x08,0x18,0x04,0x01,					//#2   1 branderpower
0x08,0x18,0x07,0x00,					//#3   2 brander aan/uit
0x08,0x18,0x07,0x50,					//#4   3 cv pomp aan/uit
0x08,0x18,0x07,0x60,					//#5   4 warmwater vraag aan/uit
0x08,0x18,0x0B,0x05,					//#6   5 ketelwatertemp x 10
0x08,0x18,0x0D,0x05,					//#7   6 retour cv water temp x 10
0x08,0x18,0x11,0x06,					//#8   7 cv waterdruk x 10
0x08,0x18,0x12,0x02,					//#9   8 status code 1ste letter
0x08,0x18,0x13,0x02,					//#10  9 status code 2de letter
0x08,0x34,0x01,0x05,					//#11 10 warmwater temp x 10
0x08,0x34,0x05,0x50,					//#12 11 boiler verwarming aan/uit
0x17,0x91,0x01,0x04,					//#13 12 ingestelde kamer temp x 2
0x17,0x91,0x02,0x05,					//#14 13 kamertemperatuur x 10
0x17,0xA8,0x17,0x81,					//#15 14 0=laag, 1=hand, 2=klok
0x17,0xA8,0x1C,0x84,					//#16 15 overruled klok instelling x 2 ( 0 = niet overruled)
0x17,0xA8,0x1D,0x84					//#17 16 ingestelde hand temp x 2
};

void printbuffer(char * buffer, int len );

char buffer[32];                				// buffer voor frame
int nefitRegister[NEFIT_REG_MAX];	            // voor de indexnummer zie boven
unsigned long register_changed =0;              // flag wanneer register veranderd is
byte pollAdres = 0;                             // als niet nul, laatste polling adres
byte nefitRegToVar[NEFIT_REG_MAX];          	// naar welke Nodo Variable moeten we schrijven ?
int regToWrite =0 ;                             // Nodig om bij Nodo eventhandling te splitsen van execution
int regValue;                                   // idem
char xmitBuffer[7] = {0x0B,0x17,0xA8,0x17,0x00,0x00,0x00};            // alvast wat defaults laden

boolean EtherComm = false;
long startTimeBrander =0;
long branderTime =0;
long startTimeWW =0;
long WWTime =0;





// bereken de CRC waarde voor de buffer
uint8_t nefit_ems_crc( char * buffer, int len ){
  uint8_t i,crc = 0x0;
  uint8_t d;
  for(i=0;i<len-2;i++){
    d = 0;
    if ( crc & 0x80 ){
      crc ^= 12;
      d = 1;
    }
    crc  = (crc << 1)&0xfe;
    crc |= d;
    crc ^= buffer[i];
  }
  return crc;
}

// check of de CRC van de buffer data klopt
boolean crcCheckOK(char * buffer, int len ){
  int crc = nefit_ems_crc(buffer, len );
  boolean crcOK= (crc==(uint8_t)buffer[len-2]);
  return crcOK;
}

// lees een data frame op de bus, geef aantal gelezen bytes terug
int readBytes(char * buffer){
  int ptr = 0;

  while(nefitSerial.available()){
    if((uint8_t)nefitSerial.peek()==0){
      uint8_t temp = nefitSerial.read();                    // skip zero's
    } else { break;}
  }

  while((!nefitSerial.frameError())&&ptr<32){          // lezen van data tot frame-error
     if(nefitSerial.available()){
      buffer[ptr]=nefitSerial.read();
      ptr++;
     }
  }
  nefitSerial.flush();
  return ptr;
}

void sendAck(){                                         // send polling acknowledge, ons adres is xmitBuffer[0]
  delay(1);
  sendBuffer(xmitBuffer,2);                             // send eigen adres en een break
}

void sendRequest(char *xmitbuffer){
  xmitBuffer[5]= nefit_ems_crc(xmitBuffer,7);           //set crc value
  pollAdres = 0;
  while ((pollAdres&0x7F)!=0x0B){                       // wachten op polling van onze interface
    int ptr = readBytes(buffer);
    if (ptr==2){ pollAdres=buffer[0];}
  }
  delay(2);                                             // 2 ms vertragen na polling, lijkt goede waarde ;-(
  sendBuffer(xmitBuffer, 7);                            // altijd 7 bytes zenden, incl break. Nog geen reden om meer/minder bytes te sturen
}

// zenden van een data frame op de bus met Nefit formaat
void sendBuffer(char * xmitBuffer, int len){
  char j;
  for(j=0;j<len-1;j++){                                   
    nefitSerial.write(xmitBuffer[j]);
    delay(3);
  }
  nefitSerial.writeEOF();
  delay(2);
  nefitSerial.flush();
}


// lees een register waarde uit de frame-buffer, decodeer indien nodig en geef terug als integer
int getValue(char * buffer, byte offset, byte vartype){
  int result;
  uint8_t type = vartype & 0x0F;
	switch (type){
		case 0:			//boolean
			result = bitRead(buffer[offset],(vartype&0x70)>>4);
			break;
		case 1:			//byte
//			result = (uint8_t)buffer[offset];
//			break;
		case 2:			//ascii
//			result = (uint8_t)buffer[offset];
//                        break;
		case 4:			//byte x 2
//			result = (uint8_t)buffer[offset];
//			break;
		case 6:			//byte x 10
			result = (uint8_t)buffer[offset];
                        break;
		case 3:			//int
//			result = getDoubleByte(buffer,offset);
//			break;
		case 5:			//int x 10
		    result = buffer[offset]<<8;
			result = result + (uint8_t)buffer[offset+1];
			break;

	}
    return result;
}

// decodeer frame mbv tabel en laad register
void nefitFrame2register(char * buffer, int len){
  byte sender, message;
  int register_data, difference;
  uint8_t offset, vartype;
  for(uint8_t i=0;i<((NEFIT_REG_MAX)*4);i=i+4){
    sender = pgm_read_byte_near(regNefitCoding+i);
    message = pgm_read_byte_near(regNefitCoding+1+i);
    if ((sender<=(uint8_t)buffer[0]) && (message<=(uint8_t)buffer[2])){
      offset = pgm_read_byte_near(regNefitCoding+2+i);
      if ((sender==(uint8_t)buffer[0]) && (message==(uint8_t)buffer[2])&&(offset>=(uint8_t)buffer[3])){
        offset = offset - (uint8_t)buffer[3]+4;       // rekening houden met lange frames (gesplitst)
        if (offset<len){
           vartype = pgm_read_byte_near(regNefitCoding+3+i);
           register_data = getValue(buffer,offset,vartype);
           if (register_data != nefitRegister[i/4])
             {
                nefitRegister[i/4] = register_data;
				if (nefitRegToVar[i/4]>0) {
				   bitSet(register_changed,i/4);     // alleen wanneer dit een Nodo var is
				}
             }
        }
      }
    }else {break;}
  }
 return;
}

// zend query command op de bus om register waarden te lezen
float queryBus(byte regnr){
  float result = 0xFFFFFFFF;
  int reg_ptr = regnr*4;
  int ptr;
  xmitBuffer[1] = pgm_read_byte_near(regNefitCoding+reg_ptr) | 0x80;    // laden bestemming met leesopdracht
  xmitBuffer[2] = pgm_read_byte_near(regNefitCoding+(reg_ptr+1));       // message type
  xmitBuffer[3] = pgm_read_byte_near(regNefitCoding+(reg_ptr+2));       // offset in bestemmings registers
  xmitBuffer[4] = 20;                                                   // !! en altijd max aantal bytes dus 20 (altijd 2 bytes vragen ?)
  sendRequest(xmitBuffer);
  long timeout = (long)millis() + 300;
  while ((((long)millis()-timeout)<0) && (!nefitSerial.available())){}        // wacht op antwoord
  if (nefitSerial.available()) {
    ptr = readBytes(buffer);
    if (ptr>4) {
      if (crcCheckOK(buffer,ptr)){
        nefitFrame2register(buffer, ptr);
//        printReg();
	if (nefitRegToVar[regnr]>0) {
	   bitSet(register_changed,regnr); 
        }
	result = nefitRegister[regnr];
      }
    }
  }
  return result;
}

//uitlezen van register en converteer formaat naar float
float nefitRegister2float(char regnr){
  float result;
  if (regnr>=0)
  {
    uint8_t format = (pgm_read_byte_near(regNefitCoding+3+(regnr*4)))&0x0F;
    switch (format){
      case 0:                                                   // uitlezen van type 0, zelfde behandeling als 1
      // result = nefitRegister[regnr]; break;
      case 1:                                                   // uitlezen van type 1, zelfde behandeling als 2
      // result = nefitRegister[regnr]; break;
      case 2:
      // result = nefitRegister[regnr]; break;                   // uitlezen van type 2, zelfde behandeling als 3
      case 3:  result = nefitRegister[regnr]; break;
      case 4:  result = nefitRegister[regnr]/2; break;
      case 5:
      //result = nefitRegister[regnr]/10; break;                 // uitlezen van type 5, zelfde behandeling als 6
      case 6:  result = (float)nefitRegister[regnr]/10.0; break;
    }
  } else { result = -1;}
  return result;
}

int writeRegister(byte regnr, int val){
  int result = 1;                                // timeout error preset
  int reg_ptr = regnr*4;
  int ptr;
 
  if (regnr<=NEFIT_REG_MAX){
    if ((pgm_read_byte_near(regNefitCoding+(reg_ptr+3))&0x80)==0x80){   // test of schrijven toegestaan
      xmitBuffer[1] = 0x17;
      xmitBuffer[2] = 0xA8;
      xmitBuffer[3] = pgm_read_byte_near(regNefitCoding+(reg_ptr+2));   // offset in bestemmings registers
      uint8_t type = pgm_read_byte_near(regNefitCoding+(reg_ptr+3))& 0x0F;
      if ((type==1) && (val<3)) {                                       // extra controle op waarde <3 (voor thermstate set)
        xmitBuffer[4] = (byte)val;
      }else if (type==4) {
        xmitBuffer[4] = (byte)val*2;
      }
      sendRequest(xmitBuffer);
      long timeout = (long)millis() + 800;
      ptr = 0;
      while ((((long)millis()-timeout)<0)&&(ptr==0)){     // wacht op antwoord
        if (nefitSerial.available()) {
          ptr = readBytes(buffer);
          if ((ptr>0)&&(buffer[0]==1)) {
            nefitRegister[regnr] = xmitBuffer[4];
            result = 0;
            break;
          } else {
           result = 2;                                               // foutcode ontvangen op bus
          }
        }
     }
    }else {
      result = 3;                                                    // not writeable
    }
  } else {
    result = 4;                                                      // fout register number
  }
  return result;
}








void setup(){
  nefitSerial.begin(9700);
  initEthernet();
  for (char i=0;i<=NEFIT_REG_MAX;i++) {
    nefitRegister[i]=0;
    nefitRegToVar[i]=0;
  }
  // hier zijn we in een paar registerwaarden geinteresseerd, set een trigger
  nefitRegToVar[2]  = 1;    // brander aan/uit
  nefitRegToVar[4]  = 2;    // wwklep aan/uit
}

void loop(){
  int ptr;
  if (pollAdres == 0) {queryBus(14);}     // forceren uitlezing van instelling thermostaat bij opstarten

  if (regToWrite > 0){
    uint8_t res=writeRegister(regToWrite,regValue);
    regToWrite = 0;
  }else if (nefitSerial.available()){  // check if we need to read
	  ptr = readBytes(buffer);
	    if (ptr == 2) {
        pollAdres = buffer[0];
//        if (pollAdres==0x8B){                                       // we worden gepolled, een acknowledge sturen
//           sendAck();                                                // hierdoor komen we vaker langs in de polling
//         }
	    } else if (ptr>4){
	        if (crcCheckOK(buffer,ptr)){
	           nefitFrame2register(buffer,ptr);
	           if (register_changed>0) {
               if (bool brander = bitRead(register_changed,2)){          // als brander-state gewijzigd
                 if (brander) {                                          // brander gaat aan
                   startTimeBrander = millis();
                 } else {                                                // brander gaat uit
                   branderTime = branderTime + millis() - startTimeBrander;
                 }
               }else if (bool wwklep = bitRead(register_changed,4))      // wwklep stand gewijzigd
                 if (wwklep) {
                   startTimeWW = millis();
                 } else {                                                // wwklep uit
                   WWTime = WWTime + millis() - startTimeWW;
                }                  
              register_changed = 0;                                     // clear change flags
	          }
	       }
	     }
    }
    if (EtherComm) {doIPData();}
}
