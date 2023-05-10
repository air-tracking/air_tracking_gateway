#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal.h>
#include <SharpGP2Y10.h>
#include <Arduino.h>
#include <string.h>

#define code_add "addme"
#define code_confirm "gotID"
#define pass "quang"
#define code_val "yourval"
#define myID 252
#define relay1 A1
#define buzzer A2
#define btr A3
#define btl A4
#define voPin A0
#define ledPin 2

byte sensor_ID[20];

float sensor_val[20];
float alert = 12;
float val_Alert[6] = {alert, 12, 35.4, 55.4, 150.4, 250.4};
const byte address[6] = "00001";

int new_ID = 0;
int new_EEPROM = 0;
int ID = 2522003;
int num_sensor = 0;
int ID_config = 0;
byte max_dust[3] = {21, 13, 40};
byte min_dust[3] = {10, 8, 30};
byte max_a = 21;
byte min_a = 10;
byte max_b = 10;
byte min_b = 0;
float dustVal = 0;

bool APreconnet = 0;
bool flag_ctrbuzz = 1;
bool auto_mode = 1;
bool relay_status = 0;
bool flag_scan = 0;
bool flag_mode = -1; // 0 = read, 1 = write
bool flag_waitAdding = 0;
bool flag_waitDeleting = 0;
bool flag_sendNRF = 0;
bool flag_updateLCD = 0;
bool flag_buzz = 0;
bool flag_connected = 0;
bool flag_sendSerial = 0;
bool flag_getDust = 0;
// bool flag_waitGetting = 0;
bool flag_got = 0;
int flag_getValNRF = 0;
int line = 0;
int flag_SSshow = 0;
byte flag_ssSend = 0;
byte flag_btr = 0;
byte flag_btl = 0;
byte flag_menu = 0;
byte flag_chAlert = 0;
byte flag_changeDust = 0;

unsigned long time_updateLCD = 3000;
unsigned long timer_updateLCD = 0;
unsigned long time_sendNRF = 500;
unsigned long timer_sendNRF = 0;
unsigned long time_waitAdding = 3000;
unsigned long timer_waitAdding = 0;
unsigned long time_waitDeleting = 3000;
unsigned long timer_waitDeleting = 0;
unsigned long time_scan = 100;
unsigned long timer_scan = 0;
unsigned long time_btr = 500;
unsigned long timer_btr = 0;
unsigned long time_btl = 500;
unsigned long timer_btl = 0;
unsigned long timer_buzz = 0;
unsigned long time_buzz = 300;
unsigned long timer_sendSerial = 0;
unsigned long time_sendSerial = 300;
unsigned long timer_getDust = 0;
unsigned long time_getDust = 1000;

RF24 radio(9, 10); // CE, CSN
LiquidCrystal lcd(7, 8, 3, 4, 5, 6);
SharpGP2Y10 dustSensor(voPin, ledPin);
bool check_Timer(unsigned long timer, unsigned long _time);
void config_value(void);
void readMode(void);
void config_Timer(void);
void timer_Control(void);
void scan_NRF(void);
void send_NRF(void);
void show_LCD(String data, bool line, bool clear);
void write_NRF(String data_write);
String read_NRF();
bool check_ID(int ID_check);
void process_data(String data_scan);
void writeMode(void);
void check_EEPROM(int val);
void save_newSensor();
void make_Request();
String get_String(String s);
void delete_someSensor(int ss_del);
void control_LCD(void);
float get_Dust(void);
void send_Serial(void);
void read_Serial(void);
void auto_Control(void);
void update_LCD(void);
void btn_Control(void);
void buzz(void);
void reset_EEPROM();

void setup()
{
  Serial.begin(9600);
  EEPROM.setMemPool(0, 1024);
  EEPROM.setMaxAllowedWrites(1024);
  reset_EEPROM();
  lcd.begin(16, 2);
  config_value();
  readMode();
  config_Timer();
}
void loop()
{
  timer_Control();
  scan_NRF();
  send_NRF();
  update_LCD();
  btn_Control();
  send_Serial();
  auto_Control();
  read_Serial();
  if (flag_getDust == 1)
  {
    flag_getDust = 0;
    dustVal = get_Dust();
  }
}
float get_Dust(void)
{
  float dustDensity = dustSensor.getDustDensity() * 1000;
  if (dustDensity < 5)
  {
    int a = random(min_a, max_a);
    int b = random(min_b, max_b);
    int c = random(min_b, max_b);
    String s = String(a) + "." + String(b) + String(c);
    return s.toFloat();
  }
  return dustDensity;
}
void send_Serial(void)
{
  if (flag_sendSerial == 1)
  {
    String data_send = "";
    flag_sendSerial = 0;

    flag_ssSend++;
    if (flag_ssSend > num_sensor)
      flag_ssSend = 0;
    if (flag_ssSend == 0)
    {
      data_send = "Data-" + String(num_sensor + 1) + "-" + String(num_sensor + 1) + "-" + String(myID) + "-" + String(dustVal) + "-" + String(num_sensor);
    }
    else
    {
      byte index = flag_ssSend - 1;
      data_send = "Data-" + String(num_sensor + 1) + "-" + String(flag_ssSend) + "-" + String(sensor_ID[index]) + "-" + String(sensor_val[index]) + "-" + String(index);
    }
    Serial.println(data_send);
  }
}
void read_Serial(void)
{
  if (Serial.available())
  {
    String data = Serial.readStringUntil('\n');
    ;
    String buff = "";
    bool flag_update = 0;
    if (data.indexOf("K-") != -1)
    {
      buff = data.substring(2, 3);
      flag_connected = buff.toInt();

      buff = data.substring(4, data.length());
      if (flag_connected)
      {
        if (buff.toFloat() != alert)
        {
          flag_update = 1;
          alert = buff.toFloat();
          EEPROM.writeFloat(300, alert);
        }
      }
      val_Alert[0] = alert;
      if ((flag_menu == 1) && (flag_update))
      {
        control_LCD();
        flag_update = 0;
      }
    }
  }
}
void auto_Control(void)
{
  auto_mode = 1;
  flag_ctrbuzz = 1;
  if (auto_mode == 1)
  {
    bool flag_check = 0;
    for (int i = 0; i < num_sensor; ++i)
    {
      if (sensor_val[i] > alert)
      {
        flag_check = 1;
        break;
      }
    }
    if (dustVal > alert)
      flag_check = 1;
    if (flag_check == 0)
      analogWrite(buzzer, 0);
    relay_status = flag_check;
  }

  if ((dustVal > alert) && (flag_ctrbuzz))
  {
    buzz();
  }
  if (!flag_ctrbuzz)
    analogWrite(buzzer, 0);
  if (digitalRead(relay1) != relay_status)
  {
    if (relay_status == 1)
      analogWrite(relay1, 1024);
    if (relay_status == 0)
      analogWrite(relay1, 0);
  }
}
void config_Timer(void)
{
  timer_btl = millis();
  timer_btr = millis();
  timer_scan = millis();
  timer_waitAdding = millis();
  timer_sendNRF = millis();
  timer_updateLCD = millis();
  timer_buzz = millis();
}
void update_LCD(void)
{
  if ((flag_waitDeleting == 0) && (flag_waitAdding == 0))
  {
    if (flag_menu == 0)
    {
      if (flag_updateLCD)
      {
        flag_updateLCD = 0;
        flag_SSshow++;
        if (flag_SSshow > num_sensor)
          flag_SSshow = 1;

        if (num_sensor == 0)
          show_LCD("no Sensor", 0, 1);
        else
        {
          show_LCD("SS " + String(flag_SSshow) + "(" + "ID-" + String(sensor_ID[flag_SSshow - 1]) + ")", 0, 1);
          show_LCD("value: " + String(sensor_val[flag_SSshow - 1]), 1, 0);
        }
      }
    }
  }
}
void control_LCD(void)
{
  if ((flag_waitDeleting == 0) && (flag_waitAdding == 0))
  {
    if (flag_menu)
    {
      if (line)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        if (relay_status)
        {
          String s = get_String((String)line + '.' + "Relay: ON");
          lcd.print(s);
        }
        else
        {
          String s = get_String((String)line + '.' + "Relay: OFF");
          lcd.print(s);
        }
        lcd.setCursor(0, 1);
        if (auto_mode)
          lcd.print((String)(line + 1) + '.' + "Auto: ON");
        else
          lcd.print((String)(line + 1) + '.' + "Auto: OFF");
      }

      if (line == 2)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        if (relay_status)
          lcd.print((String)(line - 1) + '.' + "Relay: ON");
        else
          lcd.print((String)(line - 1) + '.' + "Relay: OFF");
        lcd.setCursor(0, 1);
        if (auto_mode)
        {
          String s = get_String((String)line + '.' + "Auto: ON");
          lcd.print(s);
        }
        else
        {
          String s = get_String((String)line + '.' + "Auto: OFF");
          lcd.print(s);
        }
      }
      if (line == 3)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        String s = get_String((String)line + '.' + "APreconnet");
        lcd.print(s);
        lcd.setCursor(0, 1);
        lcd.print((String)(line + 1) + '.' + "Alert: ");
        lcd.print(alert);
      }
      if (line == 4)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print((String)(line - 1) + '.' + "APreconnet");
        lcd.setCursor(0, 1);
        String s = (String)line + '.' + "Alert: " + (String)alert;
        s = get_String(s);
        lcd.print(s);
      }
      if (line == 5)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        String s = "";
        if (flag_ctrbuzz)
          s = (String)line + '.' + "Buzz: " + "ON";
        else
          s = (String)line + '.' + "Buzz: " + "OFF";
        s = get_String(s);
        lcd.print(s);
        lcd.setCursor(0, 1);
        lcd.print((String)(line + 1) + '.' + "Quit");
      }
      if (line == 6)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        if (flag_ctrbuzz)
          lcd.print((String)(line - 1) + '.' + "Buzz: ON");
        else
          lcd.print((String)(line - 1) + '.' + "Buzz: OFF");
        lcd.setCursor(0, 1);
        String s = get_String((String)line + '.' + "Quit");
        lcd.print(s);
      }
      if (line == 7)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(get_String((String)(line) + '.' + "Delete Sensor"));
      }
    }
    if (flag_menu == 2)
    {
      String s = "";
      if (line == 1)
      {
        s = get_String("Quit");
        show_LCD(s, 0, 1);
        if (line <= num_sensor)
        {
          s = "ID-" + String(sensor_ID[line - 1]);
          show_LCD(s, 1, 0);
        }
      }
      else
      {
        if (line - 1 <= num_sensor)
        {
          if (line == 2)
          {
            s = "Quit";
            show_LCD(s, 0, 1);
            if (line - 1 <= num_sensor)
            {
              s = get_String("ID-" + String(sensor_ID[line - 2]));
              show_LCD(s, 1, 0);
            }
          }
          else
          {
            if (line % 2 != 0)
            {
              s = get_String("ID-" + String(sensor_ID[line - 2]));
              show_LCD(s, 0, 1);
              if (line <= num_sensor)
              {
                s = "ID-" + String(sensor_ID[line - 1]);
                show_LCD(s, 1, 0);
              }
            }
            if (line % 2 == 0)
            {
              s = "ID-" + String(sensor_ID[line - 3]);
              show_LCD(s, 0, 1);
              s = get_String("ID-" + String(sensor_ID[line - 2]));
              show_LCD(s, 1, 0);
            }
          }
        }
      }
    }
  }
}
void make_Request(void)
{
  if (flag_menu == 2)
  {
    if (line == 1)
    {
      flag_menu = 0;
      line = 0;
      update_LCD();
    }
    else
    {
      show_LCD("done", 0, 1);
      show_LCD("delete ID-" + String(sensor_ID[line - 2]), 1, 0);
      delete_someSensor(line - 1);

      flag_waitDeleting = 1;
      timer_waitDeleting = millis();
    }
  }
  if (flag_menu == 1)
  {
    String s = "";
    if (line == 1)
    {
      relay_status = !relay_status;
      s = "R-" + (String)relay_status;
    }
    if (line == 2)
    {
      auto_mode = !auto_mode;
      s = "A-" + (String)auto_mode;
    }
    if (line == 3)
    {
      APreconnet = 1;
      s = "N-" + (String)APreconnet;
      APreconnet = 0;
    }
    if (line == 4)
    {
      flag_chAlert++;
      if (flag_chAlert > 5)
        flag_chAlert = 0;
      alert = val_Alert[flag_chAlert];
      s = "L-" + (String)alert;
    }
    if (line == 5)
    {
      flag_ctrbuzz = !flag_ctrbuzz;
      s = "B-" + (String)flag_ctrbuzz;
    }
    if (line == 6)
    {
      flag_menu = 0;
      line = 0;
      update_LCD();
    }
    if (line == 7)
    {
      flag_menu = 2;
      line = 1;
    }
    if (line == 4)
    {
      Serial.println(s); // send gate_way
    }
  }
}
void btn_Control(void)
{
  if (!digitalRead(btr))
  {
    if (!flag_btr)
    {
      timer_btr = millis();
      ++flag_btr;
    }
  }
  if (flag_btr == 1)
  {
    if (digitalRead(btr))
    {
      if (millis() - timer_btr <= time_btr)
      {
        flag_btr++;
      }
      else
        flag_btr = 0;
    }
  }
  if (flag_btr == 2)
  {
    if (flag_menu == 0)
    {
      flag_menu = 1;
    }
    if (flag_menu == 1)
    {
      flag_btr = 0;
      ++line;
      if (line > 7)
        line = 1;
      control_LCD();
    }
    if (flag_menu == 2)
    {
      flag_btr = 0;
      ++line;
      if (line - 1 > num_sensor)
        line = 1;
      control_LCD();
    }
  }
  if (!digitalRead(btl))
  {
    if (!flag_btl)
    {
      timer_btl = millis();
      ++flag_btl;
    }
  }
  if (flag_btl == 1)
  {
    if (digitalRead(btl))
    {
      if (millis() - timer_btl <= time_btl)
      {
        flag_btl++;
      }
      else
        flag_btl = 0;
    }
  }
  if (flag_btl == 2)
  {
    flag_btl = 0;
    if (flag_menu != 0)
    {
      make_Request();
      control_LCD();
    }
    else
    {
      flag_changeDust++;
      if (flag_changeDust > 2)
        flag_changeDust = 0;
      max_a = max_dust[flag_changeDust];
      min_a = min_dust[flag_changeDust];
    }
  }
}
void scan_NRF(void)
{
  if (flag_scan)
  {
    flag_scan = 0;
    String s = read_NRF();
    if (s != "")
    {
      process_data(s);
    }
  }
}
void send_NRF(void)
{
  if (flag_sendNRF)
  {
    flag_sendNRF = 0;
    if ((flag_waitAdding == 0) && (num_sensor > 0))
    {
      flag_getValNRF++;
      if (flag_getValNRF > num_sensor)
        flag_getValNRF = 1;
      String data_write = String(pass) + "-" + String(sensor_ID[flag_getValNRF - 1]) + "-" + String(code_val) + "-" + String(alert);
      write_NRF(data_write);
    }
    else
    {
    }
  }
}
void process_data(String data_scan)
{
  // write_NRF("dataGateWay"+data_scan);
  if (data_scan.indexOf(String(pass)) != -1)
  {
    if (flag_waitAdding == 0)
    {
      if (data_scan.indexOf(String(code_add)) != -1)
      {
        // quang-addme-IDconfig
        // Serial.println("start_add");
        show_LCD("Adding...", 0, 1);
        show_LCD("please wait", 1, 0);
        flag_waitAdding = 1;
        ID_config = (data_scan.substring(12, data_scan.length())).toInt();
        String data_write = String(pass) + "-" + String(ID_config) + "-" + String(num_sensor + 1);
        write_NRF(data_write);
        timer_waitAdding = millis();
      }
      if (data_scan.indexOf(String(code_val)) != -1)
      {
        if (data_scan.indexOf(String(sensor_ID[flag_getValNRF - 1])) != -1)
        {
          // quang-ID-yourval-123;
          // Serial.println(sensor_val[flag_getValNRF-1]);
          flag_got = 1;
          sensor_val[flag_getValNRF - 1] = (data_scan.substring(data_scan.indexOf(String(code_val)) + 8, data_scan.length())).toFloat();
        }
      }
    }
    if (flag_waitAdding == 1)
    {
      if (data_scan.indexOf(String(pass)) != -1)
      {
        if (data_scan.indexOf(String(ID_config)) != -1)
        {
          if (data_scan.indexOf(String(code_confirm)) != -1)
          {
            if (check_ID(ID_config))
            {
              num_sensor++;
              sensor_ID[num_sensor - 1] = ID_config;
              save_newSensor();
              flag_waitAdding = 0;
              ID_config = 0;
              check_EEPROM(4);
              show_LCD("Add new sensor", 0, 1);
              show_LCD("ID: " + String(sensor_ID[num_sensor - 1]), 1, 0);
            }
            else
            {
              flag_waitAdding = 0;
              ID_config = 0;
              show_LCD("Can't Add ", 0, 1);
              show_LCD("SS already add", 1, 0);
            }
          }
        }
      }
    }
  }
}
bool check_ID(int ID_check)
{
  for (int i = 0; i < num_sensor; ++i)
  {
    if (sensor_ID[i] == ID_check)
      return 0;
  }
  return 1;
}
void delete_someSensor(int ss_del)
{
  for (int i = ss_del; i <= num_sensor; ++i)
  {
    EEPROM.writeByte(i, EEPROM.readByte(i + 1));
  }
  num_sensor--;
  EEPROM.writeByte(0, num_sensor);
  config_value();
  check_EEPROM(4);
}
void save_newSensor()
{
  EEPROM.writeByte(num_sensor, sensor_ID[num_sensor - 1]);
  EEPROM.writeByte(0, num_sensor);
  check_EEPROM(num_sensor + 1);
}
String read_NRF()
{
  readMode();
  char c[50] = "";
  if (radio.available())
  {
    radio.read(c, sizeof(c));
    String data = String(c);
    return data;
  }
  return "";
}
void write_NRF(String data_write)
{
  writeMode();
  const char *c = data_write.c_str();
  radio.write(c, 50);
}
void timer_Control(void)
{
  if (check_Timer(timer_scan, time_scan))
  {
    flag_scan = 1;
    timer_scan = millis();
  }
  if (check_Timer(timer_sendSerial, time_sendSerial))
  {
    flag_sendSerial = 1;
    timer_sendSerial = millis();
  }
  if (check_Timer(timer_sendNRF, time_sendNRF))
  {
    if (flag_got == 0)
    {
      sensor_val[flag_getValNRF - 1] = 0;
    }
    flag_sendNRF = 1;
    timer_sendNRF = millis();
    flag_got = 0;
  }
  if (check_Timer(timer_updateLCD, time_updateLCD))
  {
    flag_updateLCD = 1;
    timer_updateLCD = millis();
  }
  if (check_Timer(timer_getDust, time_getDust))
  {
    flag_getDust = 1;
    timer_getDust = millis();
  }
  if (millis() - timer_buzz >= time_buzz)
  {
    timer_buzz = millis();
    flag_buzz = !flag_buzz;
  }

  if (flag_waitAdding)
  {
    if (check_Timer(timer_waitAdding, time_waitAdding))
    {
      flag_waitAdding = 0;
      show_LCD("Can't Add SS", 0, 1);
      show_LCD("EEROR", 1, 0);
      timer_updateLCD = millis();
    }
  }
  if (flag_waitDeleting)
  {
    if (check_Timer(timer_waitDeleting, time_waitDeleting))
    {
      flag_waitDeleting = 0;
      line = 1;
      flag_menu = 2;
      timer_updateLCD = millis();
      control_LCD();
    }
  }
}
bool check_Timer(unsigned long timer, unsigned long _time)
{
  if (millis() >= timer)
  {
    if (millis() - timer >= _time)
    {
      return 1;
    }
  }
  if (millis() < timer)
  {
    if (60 - timer + millis() >= _time)
    {
      return 1;
    }
  }
  return 0;
}
void config_value(void)
{
  num_sensor = EEPROM.readByte(0);
  //    Serial.println("config");
  alert = EEPROM.readFloat(300);
  for (int i = 1; i <= num_sensor; ++i)
  {
    int stt = i;
    sensor_ID[i - 1] = EEPROM.readByte(stt);
    //    Serial.print(i);
    //    Serial.print(": ");
    //    Serial.println(sensor_ID[i-1]);
  }
}
void readMode(void)
{
  if (flag_mode)
  {
    flag_mode = 0;
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
  }
}
void writeMode(void)
{
  if (!flag_mode)
  {
    flag_mode = 1;
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
  }
}
void reset_EEPROM(void)
{
  for (int i = 0; i < 1024; ++i)
  {
    EEPROM.writeByte(i, 0);
  }
  Serial.println("Reset complete");
}
void check_EEPROM(int val)
{
  Serial.println("check_EEPROM");
  for (int i = 0; i < val; ++i)
  {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(EEPROM.readByte(i));
  }
}
void show_LCD(String data, bool line, bool clear)
{
  if (clear)
    lcd.clear();
  lcd.setCursor(0, line);
  lcd.print(data);
}
String get_String(String s)
{
  String re = s;
  int lg = re.length();
  lg = 16 - lg - 2;
  for (int i = 0; i < lg; ++i)
  {
    re += " ";
  }
  re += "<-";
  return re;
}
void buzz(void)
{
  if (flag_buzz == 1)
    analogWrite(buzzer, 1024);
  if (flag_buzz == 0)
    analogWrite(buzzer, 0);
}
