#include <avr/interrupt.h>
#include <EEPROM.h>
#define RDIR 11     // chiều quay động cơ phải
#define LDIR 9      // chiều quay động cơ trái
#define LPWM 5      // tốc độ động cơ trái
#define RPWM 6      // tốc độ động cơ phải
#define BUTTON1 2   // nút nhấn 1
#define BUTTON2 8   // nút nhấn 2
#define BUTTON3 7  // nút nhấn 3
#define BUZZER 4    // chân điều khiển loa
#define RED 3       // Chân RED trên mạch
#define GREEN 12     // Chân GREEN trên mạch
#define BLUE 10     // Chân BLUE trên mạch-----    // Chân BLUE trên mạch
//---------------------------------------------------
int addr = 0;
volatile int lastPos;
volatile unsigned char isCalib = 0;
volatile int servoPwm;
volatile unsigned char sensor;
unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
int speed_run_forward;
int cnt = 0;
unsigned char pattern, start;
int line = 0;
int RememberLine = 0;
float kp;
int kd;
void setup() {


  //---------------PWM------------------------//
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RDIR, LOW);
  digitalWrite(LDIR, LOW);
  digitalWrite(RED, 1);
  digitalWrite(GREEN, 1);
  digitalWrite(BLUE, 1);
  //-------------------------------------------------//
  //CÀI ĐẶT CÁC THÔNG SỐ BAN ĐẦU
  speed_run(0, 0);         // dừng 2 bánh
  pattern = 10;            // set trạng thái chạy
  start = 0;               // Trước khi chạy học màu lại, đưa cảm biến qua lại trên đường line rồi bấm SW1 để lưu
  readEeprom();
  Serial.begin(9600);
  //-------------------------------------------------//
  for (int i = 2000; i < 3500; i += 500) {
    tone(BUZZER, i, 100);
    delay(100);
  }
  timer_init();
  isCalib = 1;
  pinMode(BUZZER, OUTPUT);
  //-------------------------------------------------//
}
void timer_init() {
  // Timer/Counter 2 initialization
  ASSR = (0 << EXCLK) | (0 << AS2);
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  TCNT2 = 0xB2;
  OCR2A = 0x00;
  OCR2B = 0x00;
  // Timer/Counter 2 Interrupt(s) initialization
  TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (1 << TOIE2);
}
ISR(TIMER2_OVF_vect) {
  TCNT2 = 0xB2;
  read_sensor();
  cnt++;
}
void loop()  // vòng lặp, không dùng delay() trong này
{
  while (start == 0)  // nhấn nút để chạy
  {
    updateLine();                    
    if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để chạy
      start = 1;                      // học màu xong va bắt đầu chạy
      isCalib = 0;
      cnt = 0;
      speed_run_forward = 0;
    }
    if (digitalRead(BUTTON2) == 0) {  // Nhấn SW3 để chạy
      start = 1;                      // học màu xong va bắt đầu chạy
      isCalib = 0;
      cnt = 0;
      speed_run_forward = 130;
    }
    if (digitalRead(BUTTON3) == 0) {  // Nhấn SW3 để chạy
      start = 1;                      // học màu xong va bắt đầu chạy
      isCalib = 0;
      cnt = 0;
      speed_run_forward = 150;
    }
  }

  switch (pattern) {  // đoạn mã trạng thái máy
    case 10:
      /*if (cnt >= speed_run_forward) {  // KHởi động mềm
        pattern = 11;
        cnt = 0;
        break;
      }*/
      if (cnt >= 50) {  // KHởi động mềm
        pattern = 11;
        cnt = 0;
        break;
      }
      runforwardline(speed_run_forward);
      break;
    case 11:
     // if (sensorMask(0x81) == 0x81) {
     //   pattern = 100; 
    //    break;
    //  }
      if (sensorMask(0x01) == 0x01) {
        RememberLine = 1;
        cnt = 0;
      } else if (sensorMask(0x80) == 0x80) {
        RememberLine = -1;
        cnt = 0;
      }
      if (cnt > 500) RememberLine = 0;
      if (sensor == 0b00000000) {  // nếu bị mất line do cua gấp và có biến nhớ thì nhảy vô trường hợp giải quyết cua gấp
        if (RememberLine != 0) {
          if (RememberLine == 1) {
            handleAndSpeed(40, speed_run_forward);
          } else if (RememberLine == -1) {
            handleAndSpeed(-40, speed_run_forward);
          } else pattern = 100;                             // chuyển trạng thái sang 12 để xử lý cua gấp
        } else {                                            // Nếu mất line mà không có biến nhớ thì chạy thẳng, đoạn line đứt khúc
          speed_run(0, 0);  // chạy thẳng
        }
        break;
      } else runforwardline(speed_run_forward);  // Nếu không có gì xảy ra thì bám line bằng PID
      if (sensorMask(0b00111100) != 0b00000000) {
        RememberLine = 0;
      }
      break;
    case 12:
      if (RememberLine == 1) {
        speed_run(100, -40);
        pattern = 21;
        break;
      } else if (RememberLine == -1) {
        speed_run(-40, 100);
        pattern = 31;
        break;
      } else {
        pattern = 11;
        break;
      }
    case 21:
      speed_run(100, -40);
      RGB(1);
      if (sensorMask(0xff) != 0) {
        speed_run(60, 60 / 2);
        pattern = 22;
      }
      break; 
    case 22:
      speed_run(60, 60 / 2);
      RGB(1);
      if (sensorMask(0xfc) != 0) {
        pattern = 11;
        // RememberLine = 0;
        // line = 0;
      }
      break;


    case 31:
      speed_run(-40, 60);
      RGB(2);
      if (sensorMask(0xff) != 0) {
        speed_run(60 / 2, 60);
        pattern = 32;
      }
      break;
    case 32:
      speed_run(60 / 2, 60);
      RGB(2);
      if (sensorMask(0x3f) != 0) {
        pattern = 11;
        // RememberLine = 0;
        // line = 0;
      }
      break;
    case 100:
      speed_run(0, 0);
      RGB(millis() / 100 % 3);
      break;
    default:
      pattern = 11;
      break;
  }
}
///////////////////////////////////////////////////////////////////////
void runforwardline(int tocdo)  // hàm chạy bám line
{
  switch (sensor) {
    case 0b00000000:
      // kd = 12;
      // if (RememberLine == 1) line = 3;
      // if (RememberLine == -1) line = -3;
      // if (line > 0) {
      //   // handleAndSpeed(90, 0);
      //   speed_run(100, -40);
      //   RGB(1);
      // } else if (line < 0) {
      //   //  handleAndSpeed(-90, 0);
      //   speed_run(-40, 100);
      //   RGB(2);
      // } else {
      //   speed_run(0, 0);
      //   RGB(0);
      // }
      //speed_run(0, 0); 
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b00011000:
    case 0b00001000:
    case 0b00010000:
    case 0b00111000:
    case 0b00011100:
  
      RGB(0);
      line = 0;
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b00111100:
    case 0b00111110:
    case 0b01111110:
   
      RGB(4);
      line = 0;
      handleAndSpeed(servoPwm, tocdo);
      break;


      ///////////////////////////////////////////////////////////////////////
    case 0b00001100:
    case 0b00000100:
    case 0b00001110:
    case 0b00011110:
 
      line = 1;
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b00000110:
    case 0b00000010:
    case 0b00000111:
  
      line = 2;
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b00000011:
    case 0b00000001:
    case 0b00001111:
    case 0b00011111:
    case 0b00111111:
 
      line = 3;
      handleAndSpeed(servoPwm, tocdo);
      break;
      /////////////////////////////////////////////////////////////////////

    case 0b00110000:
    case 0b00100000:
    case 0b01110000:
    case 0b01111000:
 
      line = -1;
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b01100000:
    case 0b01000000:
    case 0b11100000:
  
      line = -2;
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b11000000:
    case 0b10000000:
    case 0b11110000:
    case 0b11111000:
    case 0b11111100:
 
      line = -3;
      handleAndSpeed(servoPwm, tocdo);
      break;

      /////////////////////////////////////////////////////////////////
    default:
      handleAndSpeed(servoPwm, tocdo);
      break;
  }
}
//--------------------------------------------------------------------------//
void updateLine() {
  //Serial.println(sensor, BIN);
  if (sensor == 0xff) beep(100);
  for (int i = 0; i < 8; i++) {
      Serial.print(sensorValue[i]);
      Serial.print("  ");
    if (black_value[i] == 0) black_value[i] = 1100;
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
   Serial.println();
  /*  if (digitalRead(BUTTON1) == 0) {
    for (int i = 0; i < 8; i++) {
      // compare_value[i] = (black_value[i] + white_value[i]) / 2;
      EEPROM.write(addr + i, compare_value[i] / 4);
    }
    digitalWrite(BUZZER, 1);
    delay(100);
    digitalWrite(BUZZER, 0);
  }*/
  RGB(millis() / 100 % 3);
}
void RGB(int color) {
  switch (color) {
    case 0:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
    case 1:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      break;
    case 2:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      break;
    default:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
  }
}
void readEeprom() {
  for (int i = 0; i < 8; i++) {
    compare_value[i] = EEPROM.read(i) * 4;
  }
}
//-------------------------------------------------------------------//

//---------------------------------------------------------------------------//
void handleAndSpeed(int angle, int speed1) {
  int speedLeft;
  int speedRight;
  if ((speed1 + angle) > 255) {
    speed1 = 255 - angle;
  }
  if ((speed1 - angle) > 255) {
    speed1 = 255 + angle;
  }
  speedLeft = speed1 + angle;
  speedRight = speed1 - angle;
  speed_run(speedLeft, speedRight);
}
//---------------------------------------------------------------------------//
void speed_run(int speedDC_left, int speedDC_right)  // hàm truyền vào tốc độ động cơ trái + phải
{

  if (speedDC_left < 0) {
    analogWrite(LPWM, 255 + speedDC_left);
    digitalWrite(LDIR, HIGH);
  } else if (speedDC_left >= 0) {
    speedDC_left = speedDC_left;
    analogWrite(LPWM, speedDC_left);
    digitalWrite(LDIR, LOW);
  }
  if (speedDC_right < 0) {
    analogWrite(RPWM, 255 + speedDC_right);
    digitalWrite(RDIR, HIGH);
  } else if (speedDC_right >= 0) {
    speedDC_right = speedDC_right;
    analogWrite(RPWM, speedDC_right);
    digitalWrite(RDIR, LOW);
  }
}
//-----------------------------------------------------------//
void read_sensor()  // hàm đọc cảm biến
{
  unsigned char temp = 0;
  unsigned int sum = 0;
  unsigned long avg = 0;
  int i, iP, iD;
  int iRet;
  //line đen
  sensorValue[0] = 1023 - analogRead(A0);
  sensorValue[1] = 1023 - analogRead(A1);
  sensorValue[2] = 1023 - analogRead(A2);
  sensorValue[3] = 1023 - analogRead(A3);
  sensorValue[4] = 1023 - analogRead(A4);
  sensorValue[5] = 1023 - analogRead(A5);
  sensorValue[6] = 1023 - analogRead(A6);
  sensorValue[7] = 1023 - analogRead(A7);
  // line trắng
  // sensorValue[0] =  analogRead(A0);
  // sensorValue[1] =  analogRead(A1);
  // sensorValue[2] =  analogRead(A2);
  // sensorValue[3] =  analogRead(A3);
  // sensorValue[4] =  analogRead(A4);
  // sensorValue[5] =  analogRead(A5);
  // sensorValue[6] =  analogRead(A6);
  // sensorValue[7] =  analogRead(A7);
  for (int j = 0; j < 8; j++) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j])
        sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j])
        sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 0, 1000);
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    } else {
      temp &= 0xfe;
    }
    sensor = temp;
  } 
  for (int j = 0; j < 8; j++) {
    avg += (long)(sensorPID[j]) * ((j)*1000);
    sum += sensorPID[j];
  }
  i = (int)((avg / sum) - 3500);
  kp = 1.25 ;
  kd = 3;
  iP = kp * i;
  iD = kd * (lastPos - i);
  iRet = (iP - iD);
  if ((iRet < -4000)) {
    iRet = 0;
  }
  servoPwm = iRet / 75;//30 Banh 68mm
  lastPos = i;
}
void beep(int timer) {
  digitalWrite(BUZZER, 1);
  delay(timer);
  digitalWrite(BUZZER, 0);
}
unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}
int check90() {
  if (sensorMask(0x0f)) return 1;
  if (sensorMask(0xf0)) return -1;
}
