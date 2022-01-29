#include <IRremote.h>
#include <Mouse.h>
#include <Keyboard.h>
#include <EEPROM.h>
#include <avr/wdt.h>

union block
{
  uint32_t a;
  byte b[4];
};

const int PIN = 11;//红外
const int PWRBTTN = MISO; //接受控设备开关
const int LED = 13;
union block readbuffer;
byte bufferpos = 0;
bool filled = false;
IRrecv HW(PIN);//声明一个IRrecv类型的对象，并接到PIN，也就是11脚
uint32_t results;//声明一个decode_results类型的对象results,存放红外接收的信号解码结果
int blinkspd = 0;
int lightlev = 1;
int maxlevel = 255;// used as an EEP pointer when in EEProgramMode.
bool eepprogram = false;
bool runeep = false;
bool isdown[259];
byte lastexecuteaddr = 0;
unsigned long lastexecute;

//EEPROM MODE
bool hyper = false;


void setup() {
  for (int i = 0; i < 259; i++) {
    isdown[i] = false;
  }
  Serial.begin(9600);
  //Serial.setTimeout(500);
  pinMode(PWRBTTN, OUTPUT);
  pinMode(LED, OUTPUT);
  //digitalWrite(PWRBTTN, LOW);
  digitalWrite(LED, HIGH);
  runeep = IsEEPCmdAvaliable();
  HW.enableIRIn();
  Keyboard.begin();
  Mouse.begin();
  if (runeep) {
    Serial.println("EEPROM");
    uint32_t data = ReadEEPBlock(0);
    RunCommand(data);
    lastexecute = millis();
  }
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  unsigned long time1 = millis();
  if (HW.decode())
  {
    results = HW.decodedIRData.decodedRawData;
    if (!eepprogram) {
      analogWrite(LED, lightlev);
      union block align, reverse;
      align.a = results;
      reverse.b[0] = align.b[3];
      reverse.b[1] = align.b[2];
      reverse.b[2] = align.b[1];
      reverse.b[3] = align.b[0];

      Serial.print("!");
      Serial.println(results, HEX);
      if (!(runeep && RunEEPROMCommand(reverse.a))) {
        Serial.println(results, HEX);
      }
      /*
        switch (results)
        {
        case 0xBF40FF00:
          digitalWrite(PWRBTTN, HIGH);
          delay(250);
          digitalWrite(PWRBTTN, LOW);
          break;
        }
      */

      HW.resume();//接收下一个红外信号
      delay(25);
      analogWrite(LED, 255 - lightlev);
    }
  }
  if (Serial.available())serialEvent();
  if (filled) {//接收缓冲区4字节满
    filled = false;
    //普通模式
    if (!eepprogram) {
      RunCommand(readbuffer.a);
    } else {
      //EEPORM配置模式
      WriteEEPBlock(maxlevel, readbuffer.a);
      Serial.println(maxlevel, HEX);
      maxlevel++;
      if (maxlevel >= 256) {
        Serial.println("OVERFLOW");
        EEPDUMP();
      }
      if (readbuffer.a == 0xCCCCCCCC) {
        Serial.println("EEPDUMP");
        EEPDUMP();
        runeep = IsEEPCmdAvaliable();
      }
    }
  }
  unsigned long time2 = millis();
  int delta = time2 > time1 ? time2 - time1 : (4294967295 - time1) + time2;
  if (delta < 40) {
    delta = 40 - delta;
    delay(delta);
  }
  if (blinkspd != 0) {
    lightlev += blinkspd;
    if (lightlev >= maxlevel || lightlev <= 0) {
      blinkspd = -blinkspd;
    }
    if (lightlev > maxlevel)lightlev = maxlevel;
    if (lightlev < 0)lightlev = 0;
    analogWrite(LED, 255 - lightlev);
  }
}

bool RunEEPROMCommand(uint32_t IR) {
  int now = millis();
  int delta = now > lastexecute ? now - lastexecute : (4294967295 - lastexecute) + now;
  lastexecute = now;
  if (IR == 0 && delta < 1000) {
    RunEEPROMCommandBlockAt(lastexecuteaddr);
  }
  uint32_t data;
  byte addr = 1;
  int nullcnt = 0;
  do
  {
    do
    {
      data = ReadEEPBlock(addr);
      addr++;
    } while ((((!hyper) && (data != 0)) || ((hyper) && (data != 0xFFFFFFFF))) && (addr < 255) && (data != 0xCCCCCCCC));
    if (data != 0xCCCCCCCC) {
      data = ReadEEPBlock(addr);
      addr++;
      if (data == IR) {
        Serial.print("!Run at:");
        Serial.println(addr, HEX);
        RunEEPROMCommandBlockAt(addr);
        return true;
      }
    }
  } while (data != 0xCCCCCCCC && addr < 255);
  return false;
}

void RunEEPROMCommandBlockAt(byte commandstart) {
  uint32_t data;
  byte addr = commandstart;
  lastexecuteaddr = commandstart;
  //Run commandList
  do {
    data = ReadEEPBlock(addr);
    addr++;
    Serial.print("!CMD:");
    Serial.println(data, HEX);
    RunCommand(data);
    if (addr >= 256)break;
  } while (data != 0 && data != 0xFFFFFFFF && data != 0xCCCCCCCC);
}

void EEPDUMP() {
  int addr = 0;
  uint32_t data;
  do
  {
    data = ReadEEPBlock(addr);
    addr++;
    Serial.println(data, HEX);
  } while (data != 0xCCCCCCCC);
}

void RunCommand(uint32_t cmd) {
  union block data;
  data.a = cmd;
  switch (data.b[0]) {
    case 0xBF:
      Keyboard.press(data.b[1]);
      break;
    case 0xC0:
      Keyboard.release(data.b[1]);
      break;
    case 0xC1:
      Keyboard.write(data.b[1]);
      break;
    case 0xC2:
      Mouse.move(data.b[1] - 127, data.b[2] - 127, data.b[3] - 127);
      break;
    case 0xC3:
      Mouse.press(data.b[1]);
      break;
    case 0xC4:
      Mouse.release(data.b[1]);
      break;
    case 0xC5://Change light brightness
      analogWrite(LED, 255 - data.b[1]);
      lightlev = data.b[1];
      maxlevel = data.b[1];
      blinkspd = 0;
      break;
    case 0xC6://Change light breath rate
      blinkspd = data.b[1];
      maxlevel = data.b[2];
      break;
    case 0xC7://Switch on/off hyper mode
      hyper = !hyper;
      if (runeep) {
        lightlev = hyper ? 255 : 1;
        maxlevel = lightlev;
        blinkspd = 0;
        Serial.println(hyper ? "HYPER" : "DEHYPER");
      }
    case 0xC8:
      Mouse.click(data.b[1]);
      break;
    case 0xC9://Switch key
      if (isdown[data.b[1]])Keyboard.release(data.b[1]);
      else Keyboard.press(data.b[1]);
      isdown[data.b[1]] = !isdown[data.b[1]];
      break;
    case 0xCA://Switch mouse
      if (isdown[data.b[1] + 254])Mouse.release(data.b[1]);
      else Mouse.press(data.b[1]);
      isdown[data.b[1] + 254] = !isdown[data.b[1] + 254];
      break;
    case 0xCB://High puls
      if (data.b[1] > 0)digitalWrite(PWRBTTN, HIGH);
      delay(data.b[1]);
      digitalWrite(PWRBTTN, LOW);
      break;
    case 0xCC://Low puls
      if (data.b[1] > 0)digitalWrite(PWRBTTN, LOW);
      delay(data.b[1]);
      digitalWrite(PWRBTTN, HIGH);
      break;
    case 0xCD:
      RunEEPROMCommandBlockAt(lastexecute);
      break;
    case 0xDD://Program EEPROM
      eepprogram = true;
      maxlevel = 0;
      break;
    case 0xA0://JUMP
      if (runeep)RunEEPROMCommandBlockAt(data.b[1]);
      break;
    case 0xEE://Disable EEPROM
      runeep = false;
      break;
    case 0xEF://Enable EEPROM if available
      runeep = IsEEPCmdAvaliable();
      break;
    case 0xF0:
      if (runeep)Serial.println("EEPENABLED");
      break;
  }
}

bool IsEEPCmdAvaliable() {
  return ReadEEPBlock(0) != 0x00000000;
}

uint32_t ReadEEPBlock(int addr) {
  if (addr >= 256)return 0;
  union block data;
  data.b[0] = EEPROM.read(addr * 4);
  data.b[1] = EEPROM.read(addr * 4 + 1);
  data.b[2] = EEPROM.read(addr * 4 + 2);
  data.b[3] = EEPROM.read(addr * 4 + 3);
  return data.a;
}

void WriteEEPBlock(int addr, uint32_t val) {
  union block data;
  data.a = val;
  EEPROM.write(addr * 4, data.b[0]);
  EEPROM.write(addr * 4 + 1, data.b[1]);
  EEPROM.write(addr * 4 + 2, data.b[2]);
  EEPROM.write(addr * 4 + 3, data.b[3]);
}

void serialEvent() {
  while (Serial.available()) {
    readbuffer.b[bufferpos] = Serial.read();
    bufferpos++;
    if (bufferpos >= 4) {
      filled = true;
      bufferpos = 0;
      break;
    } else {
      filled = false;
    }
  }
}
