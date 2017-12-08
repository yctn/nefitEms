/* Hierin de functies de het mogelijk maken om een Ethernet UDP syslog message te triggeren tbv data opslag.

Richting syslog worden, wanneer er om gevraagd wordt (iedere 10min met "R10"), de volgende waarden gedumpt:
keteltemp, wateruittemp, waterretourtemp, kamertemp, ingestelde kamertemp, thermostaat-stand, waterdruk, hoelang brander aan sinds laatste uitlezing, hoelang warmwater..

Verder kunnen er met TCP/IP de volgende opdrachten gegeven worden:
State=<val>    hierbij is <val>: 0=vorst, 1=hand, 2 = klok
TempSet=<temp> stel in op de aangegeven temperatuur in <temp>. Als state=1 wordt dit de handtemp, anders wordt de kloktemp overruled.
               met TempSet=0 (in stand 2) wordt de overrulde kooktemp ongedaan gemaakt.
State?         opvragen huidige thermostaat stand, en kamertemp.

In princiepe moet e.e.a. ook zonder deze module werken, het is een optie

*/

#include <EthernetNodo.h>
#include <EthernetUDP.h>
IPAddress remoteIP(192,168,178,150);
int remotePort	= 514;
IPAddress ackIP(0,0,0,0); // Deze variabele wordt niet gebruikt.. we gebruiken DHCP voor eigen adres van arduino ethernet
int serverPort = 5123;    // deze wordt gebruikt voor TCP, voor UDP wordt 1022 gebruikt

byte mac[] = {  
  0xCC, 0xBB, 0xAA, 0x31, 0x30, 0x37 };			// set MAC address in NODO range
  
char *packetBuffer=(char*)malloc(UDP_TX_PACKET_MAX_SIZE);

int writeRegister(byte regnr, int val);
  
EthernetUDP  Udp;								// an instance to send packets
EthernetServer server(serverPort);

void doSendHeatingData(){
  Udp.beginPacket(remoteIP,remotePort);
  Udp.print("<134>Apr 1 11:11:11 NEFIT 10 ");
  Udp.print((float)nefitRegister[5]/10,1);    //keteltemp
  Udp.print(",");
  Udp.print((float)nefitRegister[6]/10,1);    //waterterug temp
  Udp.print(",");
  Udp.print((float)nefitRegister[0]/10,1);    //wateruit
  Udp.print(",");
  Udp.print((float)nefitRegister[10]/10,1);   //warmwatertemp
  Udp.print(",");
  Udp.print((float)nefitRegister[13]/10,1);   // huidige kamertemp
  Udp.print(",");
  Udp.print((float)nefitRegister[12]/2,1);   //ingestelde kamer temp
  Udp.print(",");
  Udp.print(nefitRegister[14],DEC);           //stand thermostaat 0=laag, 1=hand, 2=autot
  Udp.print(",");
  Udp.print((float)nefitRegister[7]/10,1);    // waterdruk
  Udp.print(",");
  if (nefitRegister[2]==1) {
    branderTime = millis() - startTimeBrander;
    startTimeBrander = millis();
  }
  if (branderTime>570000) {branderTime=600000;}
  Udp.print((float)branderTime/60000);
  branderTime = 0;
  Udp.print(",");
  if (nefitRegister[4]==1) {
    WWTime = millis() - startTimeWW;
    startTimeWW = millis();
  }
  if (WWTime>570000) {WWTime=600000;}
  Udp.print((float)WWTime/60000);
  WWTime = 0;
	Udp.endPacket();

} 

void initEthernet(){
  if (EthernetNodo.begin(mac)==0) {
    } else {
    EtherComm = true;
    }
  Udp.begin(1022);                                    // open port 1022
}



//*********************************************************************************************************************
void doIPData(){ // check if we have a request for data, if not return else provide it (if it makes sense)

  int s = Udp.parsePacket();
  int val;

  if (s>2){
     String tmpstr;
     Udp.read(packetBuffer,s);
     packetBuffer[s]=0x00;
     String str(packetBuffer);

     
     if ((str=="R10")&&(nefitRegister[0]>0)) {           // received R10 a message that shows up every 10min
       doSendHeatingData();
     } 

  }
  EthernetClient client = server.available();
  if (client){
     int i =0;
     while (client.available() >0){
       packetBuffer[i++]=client.read();
       if (i>19) {break;}
     }
     packetBuffer[i] = 0x0;

     String str(packetBuffer);
     String tmpstr;
     int res = -1;                                // default command error
     if (str.startsWith("State=")){
        tmpstr = str.substring(6);
        val = tmpstr.toInt();
        if ((val>0)&&(val<3)){
            res=writeRegister(14, val);
        } else { res=-2;}                          // parameter error
      } else if (str.startsWith("TempSet=")){
        tmpstr = str.substring(8);
        val = tmpstr.toInt();
        if ((val==0) || ((val>10)&&(val<23))){
          if (nefitRegister[14]==1){                //we staan in handbediening, schrijven hand-temp
             res=writeRegister(16, val);
          } else {
             res=writeRegister(15, val);            // anders override kloktemp
          }
        } else { res=-2;}
      } else if (str.startsWith("State?")){
        float tmpVal;
        char resultStr[10];
        res = 0;
        switch (nefitRegister[14]){
          case 0: tmpstr="LAAG"; break;
          case 1: tmpVal=nefitRegister[16]/2;
                  tmpstr="HAND = ";
                  dtostrf(tmpVal,4,1,resultStr);
                  tmpstr=tmpstr + resultStr;
                  break;
          case 2: tmpstr="KLOK";
                  if (nefitRegister[15]>0){
                    tmpVal=nefitRegister[15]/2;
                    dtostrf(tmpVal,4,1,resultStr);
                    tmpstr=tmpstr+" overruled: "+resultStr;
                  }
        }
        server.println(tmpstr);server.println();
        tmpVal = nefitRegister[13]/10;
        dtostrf(tmpVal,4,1,resultStr);
        tmpstr="Kamertemp nu: ";
        tmpstr=tmpstr+resultStr;
        server.println(tmpstr);
        server.println();
      }
      if (res==0) {
         server.println("OK");
         server.println();
      } else {
         server.print("error:");
         server.println(res);
         server.println();
      }
  }
}

