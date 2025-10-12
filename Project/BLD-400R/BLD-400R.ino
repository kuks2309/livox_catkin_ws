#include <Arduino.h>
#include <SPI.h>

#ifdef ARDUINO_AVR_MEGA2560
  // Serial1은 여기서 자동으로 정의됨
#endif

#include <MsTimer2.h>

#define NO_DATA                     11
#define MODBUS_BUFFER_SIZE          13
#define UART_BUFFER_SIZE            13
#define DEBUG_ENABLE                false 
#define STEER_ANGLE_TO_PULSE        166
#define PULSE_TO_STEER_ANGLE        1./STEER_ANGLE_TO_PULSE
#define Angle_To_Encoder_Conversion 14502

#define Fornt_Steering_ADC_LEFT     200   //측정해서 입력해야함
#define Fornt_Steering_ADC_CENTER   468   //측정해서 입력해야함  2호기
#define Fornt_Steering_ADC_RIGHT    498   //측정해서 입력해야함

#define Rear_Steering_ADC_LEFT      200   //측정해서 입력해야함
#define Rear_Steering_ADC_CENTER    410   //측정해서 입력해야함 2호기
#define Rear_Steering_ADC_RIGHT     700   //측정해서 입력해야함

// RS485 TX Control

#define TX_485_1 38
#define TX_485_2 30
#define TX_485_3 29

// Limit Switch Bank1

#define D42   42  
#define D43   43
#define D44   44
#define D45   45
#define D46   46
#define D47   47


// Limit Switch Bank2   
//D22 :Front Steering Left
//D23 :Frint Steering Right
//D24 :Rear Steering Left
//D25 :Rear Steering Right

#define D22  22    
#define D23  23    
#define D24  24    
#define D25  25    
#define D26  26    
#define D27  27

// Interrupt Pin

#define INT_1 2
#define INT_2 3


#define MANUAL    0 
#define SEMI_AUTO 1 
#define FULL_AUTO 2 

#define CRAB_MODE 10
#define TANK_MODE 20
#define CAR_MODE  30


// SPI PIN
const int SS_PIN = 10;

struct Protocol {
  byte header = 0xAA;    // 시작 바이트
  byte mode;             // Mode-manual/mode_car
  byte ch1_H;           // channel1 상위 바이트
  byte ch1_L;           // channel1 하위 바이트
  byte ch2_H;           // channel2 상위 바이트
  byte ch2_L;           // channel2 하위 바이트
  byte ch3_H;           // channel3 상위 바이트
  byte ch3_L;           // channel3 하위 바이트
  byte ch4_H;           // channel4 상위 바이트
  byte ch4_L;           // channel4 하위 바이트
  byte crc;             // CRC 체크섬
};

Protocol sendProtocol;
Protocol receiveProtocol;

// 현재 모드 정보 저장 변수
byte steering_mode = CRAB_MODE;
byte run_mode      = MANUAL;


// CAN 통신
//MCP_CAN CAN0(10);

// 스티어링 모터 상태를 위한 변수
struct SteeringState 
{
  bool is_moving;              // 현재 이동 중인지
  float target_angle;          // 목표 각도
  long target_encoder;         // 목표 엔코더 값
  unsigned long start_time;    // 이동 시작 시간
  unsigned long timeout;       // 타임아웃 (ms)
  bool wheels_disabled;        // 휠 모터 비활성화 상태
};

SteeringState front_steering_state = {false, 0, 0, 0, 5000, false}; // 5초 타임아웃
SteeringState rear_steering_state  = {false, 0, 0, 0, 5000, false};  // 5초 타임아웃

// 전역 변수로 차동 속도 선언
int16_t diff_speed = 0;
int16_t base_speed = 0;
// flag 

static bool wheel_speed_command_update[4]    = {false,false,false,false};
static bool steering_angle_command_update[2] = {false,false};

unsigned char modbus_buf1[MODBUS_BUFFER_SIZE];
unsigned char modbus_buf2[MODBUS_BUFFER_SIZE];
unsigned char modbus_buf3[MODBUS_BUFFER_SIZE];
unsigned char readBuf[UART_BUFFER_SIZE];
int modbus_buf_index = 0;
static int command_type = 0;
static int robot_run_mode       = 0;   // manual, semi_auto, full_auto                  
static int robot_steering_mode  = 0;   // crab or tank mode

float system_voltage[11]                    = {0.0,};
int32_t encoder_pos_wheel[4]                = {0};
int32_t encoder_pos_fork[4]                 = {0};
int32_t encoder_pos_steering[2]             = {0};

static int16_t wheel_motor_speed[4]         = {0, 0, 0, 0 };
static int16_t steering_angle[2]            = {0, 0};

bool fork_limit_switch_status[8]            = {0, 0, 0, 0, 0, 0, 0, 0};
bool front_steering_limit_switch_status[2]  = {0, 0};
bool rear_steering_limit_switch_status[2]   = {0, 0};


// Define variables to track consecutive mode values
byte last_run_mode = 255; // Initialize to an invalid value
byte consecutive_count = 0;


// Timer 구현
unsigned long int preTime = 0;
unsigned long int interval = 100;

void motor_all_stop(void);
void isWheelSpeedCommandUpdated(int16_t *wheel_motor_speed_new);

// CRC16 테이블
static const uint16_t wCRCTable[] = 
                                    { 0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780, 
                                      0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 
                                      0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 
                                      0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 
                                      0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 
                                      0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 
                                      0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 
                                      0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 
                                      0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981, 
                                      0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 
                                      0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 
                                      0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 
                                      0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 
                                      0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 
                                      0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 
                                      0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 
                                      0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 
                                      0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 
                                      0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 
                                      0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40, 
                                      0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 
                                      0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 
                                      0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 
                                      0X81C1, 0X8081, 0X4040 };


#define WINDOW_SIZE 6

class RecursiveMovingAverage 
{
    private:
        float sensorData[WINDOW_SIZE];
        int windowSize;
    
    public:
        RecursiveMovingAverage() : windowSize(WINDOW_SIZE) 
        {
            for (int i = 0; i < windowSize; i++) 
            {
                sensorData[i] = 0.0;
            }
        }
    
        float update(float adValue, float avgOld) 
        {
            float avg = 0.0;
            float oldValue = sensorData[0];
    
            // Shift the array
            for (int i = 0; i < windowSize - 1; i++) 
            {
                sensorData[i] = sensorData[i + 1];
            }
            sensorData[windowSize - 1] = adValue;
    
            // Calculate new average
            avg = avgOld + (sensorData[windowSize - 1] - oldValue) / windowSize;
    
            return avg;
        }
};


RecursiveMovingAverage movingAvg1;
RecursiveMovingAverage movingAvg2;


// 16비트 채널값을 상위/하위 바이트로 분리하는 함수
void setChannelValue(int channelValue, byte &highByte, byte &lowByte) 
{
  highByte = (channelValue >> 8) & 0xFF;
  lowByte = channelValue & 0xFF;
}

// CRC 계산 함수
byte calculateCRC(Protocol &p) 
{
  byte crc = p.header ^ p.mode;
  crc ^= p.ch1_H ^ p.ch1_L;
  crc ^= p.ch2_H ^ p.ch2_L;
  crc ^= p.ch3_H ^ p.ch3_L;
  crc ^= p.ch4_H ^ p.ch4_L;
  return crc;
}

void sendData_SPI() 
{
  // 예시 데이터 설정
  sendProtocol.mode = 0x01;  // manual mode
  
  // 채널값 설정 (예시: 16비트 값)
  setChannelValue(1500, sendProtocol.ch1_H, sendProtocol.ch1_L);  // 채널1: 1500
  setChannelValue(1000, sendProtocol.ch2_H, sendProtocol.ch2_L);  // 채널2: 1000
  setChannelValue(2000, sendProtocol.ch3_H, sendProtocol.ch3_L);  // 채널3: 2000
  setChannelValue(1750, sendProtocol.ch4_H, sendProtocol.ch4_L);  // 채널4: 1750
  
  // CRC 계산
  sendProtocol.crc = calculateCRC(sendProtocol);

  // 데이터 전송 및 응답 수신
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(100);
  
  receiveProtocol.header = SPI.transfer(sendProtocol.header);
  delayMicroseconds(10);
  receiveProtocol.mode = SPI.transfer(sendProtocol.mode);
  delayMicroseconds(10);
  receiveProtocol.ch1_H = SPI.transfer(sendProtocol.ch1_H);
  delayMicroseconds(10);
  receiveProtocol.ch1_L = SPI.transfer(sendProtocol.ch1_L);
  delayMicroseconds(10);
  receiveProtocol.ch2_H = SPI.transfer(sendProtocol.ch2_H);
  delayMicroseconds(10);
  receiveProtocol.ch2_L = SPI.transfer(sendProtocol.ch2_L);
  delayMicroseconds(10);
  receiveProtocol.ch3_H = SPI.transfer(sendProtocol.ch3_H);
  delayMicroseconds(10);
  receiveProtocol.ch3_L = SPI.transfer(sendProtocol.ch3_L);
  delayMicroseconds(10);
  receiveProtocol.ch4_H = SPI.transfer(sendProtocol.ch4_H);
  delayMicroseconds(10);
  receiveProtocol.ch4_L = SPI.transfer(sendProtocol.ch4_L);
  delayMicroseconds(10);
  receiveProtocol.crc = SPI.transfer(sendProtocol.crc);
  delayMicroseconds(10);
  
  digitalWrite(SS_PIN, HIGH);
  delayMicroseconds(100);  

  parseMode(receiveProtocol.mode);

  if(receiveProtocol.mode!=22)
  {
    digitalWrite(TX_485_3, HIGH);
    Serial3.print("Steering Mode :");
    Serial3.println(steering_mode);
    Serial3.print("Run Mode:");
    Serial3.println(run_mode);
  }
   
  // 데이터 출력
  //Serial.println("Sent data:");
  //printProtocol(sendProtocol);
  
  //Serial.println("Received response:");
  //printProtocol(receiveProtocol);
}

// 모드 값 파싱 함수
// Define variables to track consecutive mode values

void parseMode(byte mode) 
{
  // Extract run_mode (1's place)
  byte potential_run_mode = mode % 10;  
  
  // Check if this is the same run_mode as last time
  if (potential_run_mode == last_run_mode) 
  {
    consecutive_count++;
    
    // Only change the actual run_mode if we've seen the same value 3 or more times
    if (consecutive_count >= 3) 
    {
      // Set the run_mode based on the consistent input
      switch (potential_run_mode) 
      {
        case 0:
          run_mode = MANUAL;
          break;  
        case 1:
          run_mode = SEMI_AUTO;         
          break;    
        case 2:
          run_mode = FULL_AUTO;         
          break;    
        default:
          run_mode = FULL_AUTO;         
          break;
      }
      
      // Reset the counter after making the change
      consecutive_count = 0;
    }
  }
  else 
  {
    // Different mode than before, reset the counter
    last_run_mode = potential_run_mode;
    consecutive_count = 1;
  }
  
  // Extract steering_mode (10's place) - this doesn't need the consecutive logic
  steering_mode = (mode / 10) * 10;  // Get the 10's place and multiply by 10
  
  switch (steering_mode) 
  {
    case 10:
      steering_mode = CRAB_MODE;
      break;  
    case 20:
      steering_mode = TANK_MODE;         
      break;    
    case 30:
      steering_mode = CAR_MODE;         
      break;    
    default:
      steering_mode = TANK_MODE;         
      break;    
  }
}


void printProtocol(Protocol &p) 
{
  Serial.print("Header: 0x"); Serial.print(p.header, HEX);
  Serial.print(" Mode: 0x"); Serial.print(p.mode, HEX);
  Serial.print(" CH1: "); Serial.print((p.ch1_H << 8) | p.ch1_L);
  Serial.print(" CH2: "); Serial.print((p.ch2_H << 8) | p.ch2_L);
  Serial.print(" CH3: "); Serial.print((p.ch3_H << 8) | p.ch3_L);
  Serial.print(" CH4: "); Serial.print((p.ch4_H << 8) | p.ch4_L);
  Serial.print(" CRC: 0x"); Serial.println(p.crc, HEX);
}

//ADC 관련 함수
float read_front_steering_sensor_adc(void)
{
    static float avgOld1 = 0;
    float ad_value;
    float adValue = (float)analogRead(A1); // Read analog value
    //Serial.print("ad1 value : ");
    //Serial.println(analogRead(A0));
    //Serial.println(avgOld1);
    //Serial.println("");
    ad_value = movingAvg1.update(adValue, avgOld1);
    avgOld1 = ad_value;
    return ad_value;
}


float read_rear_steering_sensor_adc(void)
{

  static float avgOld2 = 0.0;
  float ad_value;
  float adValue = (float)analogRead(A2); // Read analog value; 

  //Serial.println(avgOld2);
  //Serial.println(adValue);
  //Serial.println("");   
  ad_value = movingAvg2.update(adValue, avgOld2);
  avgOld2 = ad_value;
  
  return ad_value;
}


float read_front_steering_sensor(void)
{
   float angle = 0.0;
   float read_front_adc = read_front_steering_sensor_adc();

   if(read_front_adc <= Fornt_Steering_ADC_CENTER)
   {
      angle = map( read_front_adc , Fornt_Steering_ADC_LEFT, Fornt_Steering_ADC_CENTER, 90, 0);
   }  
   else
   {
      angle = map( read_front_adc , Fornt_Steering_ADC_CENTER, Fornt_Steering_ADC_RIGHT, 90, 0);
   }
   return angle;
}

float read_rear_steering_sensor(void)
{
  float angle = 0.0;
  float read_front_adc = read_rear_steering_sensor_adc();

  if(read_front_adc <= Fornt_Steering_ADC_CENTER)
  {
    angle = map( read_front_adc , Fornt_Steering_ADC_LEFT, Fornt_Steering_ADC_CENTER, 90, 0);
  }  
  else
  {
    angle = map( read_front_adc , Fornt_Steering_ADC_CENTER, Fornt_Steering_ADC_RIGHT, 90, 0);
  }
    return angle;
}


// check front steering 

void check_front_steering(void)
{

  float front_steer_neutral_adc = Fornt_Steering_ADC_CENTER;

  float average_front_steering_adc ;
  float error =  0;
  int corrected_steering_angle = 0;

  do
  {
    average_front_steering_adc = read_front_steering_sensor_adc();
    
    error = front_steer_neutral_adc - read_front_steering_sensor_adc();
    if(error >=0 )
    {
      corrected_steering_angle += 1;//부호는 방향에 맞도록 수정해야 함
      front_steering_control(corrected_steering_angle); 
      delay(500);
    }

    else
    {
      corrected_steering_angle -= 1;  //부호는 방향에 맞도록 수정해야 함
      front_steering_control(corrected_steering_angle); 
      delay(500);
    }

  }while(fabs(error)>2);
  
  Serial.println("corrected_steering_angle");

}

void check_rear_steering(void)
{

  float rear_steer_neutral_adc = Rear_Steering_ADC_CENTER;

  float average_rear_steering_adc ;
  float error =  0;
  int corrected_steering_angle = 0;

  do
  {
    average_rear_steering_adc = read_rear_steering_sensor_adc();
    
    error = rear_steer_neutral_adc - read_rear_steering_sensor_adc();
    if(error >=0 )
    {
      corrected_steering_angle += 1;//부호는 방향에 맞도록 수정해야 함
      rear_steering_control(corrected_steering_angle); 
      delay(200);
    }

    else
    {
      corrected_steering_angle -= 1;  //부호는 방향에 맞도록 수정해야 함
      rear_steering_control(corrected_steering_angle); 
      delay(200);
    }

  }while(fabs(error)>2);
  
  Serial.println("corrected_steering_angle");

}

// RS485 송신 함수

void RS485_Send1(uint8_t* data, size_t length) 
{
    digitalWrite(TX_485_1, HIGH);    // 송신 모드로 전환
    //delayMicroseconds(10);          // 모드 전환을 위한 짧은 대기 시간
    
    Serial1.write(data, length);     // 데이터 송신
    Serial1.flush();                 // 송신 완료 대기
    
    delayMicroseconds(30);          // 송신 완료 후 짧은 대기 시간 25가 최소(실험에서 찾았음)
    digitalWrite(TX_485_1, LOW);     // 수신 모드로 전환
}


void RS485_Send2(uint8_t* data, size_t length) 
{
    digitalWrite(TX_485_2, HIGH);    // 송신 모드로 전환
    //delayMicroseconds(10);          // 모드 전환을 위한 짧은 대기 시간
    
    Serial2.write(data, length);     // 데이터 송신
    Serial2.flush();                 // 송신 완료 대기
    
    delayMicroseconds(30);          // 송신 완료 후 짧은 대기 시간 25가 최소(실험에서 찾았음)
    digitalWrite(TX_485_2, LOW);     // 수신 모드로 전환
}


void RS485_Send3(uint8_t* data, size_t length) 
{
    digitalWrite(TX_485_3, HIGH);    // 송신 모드로 전환
    //delayMicroseconds(10);          // 모드 전환을 위한 짧은 대기 시간
    
    Serial3.write(data, length);     // 데이터 송신
    Serial3.flush();                 // 송신 완료 대기
    
    delayMicroseconds(30);          // 송신 완료 후 짧은 대기 시간 25가 최소(실험에서 찾았음)
    digitalWrite(TX_485_3, LOW);     // 수신 모드로 전환
}

// CRC16 MODBUS 계산 함수
uint16_t CRC16_MODBUS(const uint8_t *nData, uint16_t wLength) 
{
  uint8_t nTemp;
  uint16_t wCRCWord = 0xFFFF;
  
  while (wLength--) 
  {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}

// 16진수 출력 헬퍼 함수
void printHex(uint8_t num) 
{
  if (num < 0x10) Serial.print("0");
  Serial.print(num, HEX);
}


// Modbus Enable 명령 전송 함수
void sendModbusEnable(uint8_t RS485_channel, uint8_t motor_id, bool enabled) 
{
  command_type = 0;
  // 명령 프레임 구성: [ID + Function Code + Register Address + Data]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x06,       // Function Code (Write Single Register)
    0x00, 0x00, // Register Address (ModbusEnable: 0x0000)
    0x00, enabled  // Data (Enable: 0x0001)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;

  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
  
  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(6500);  // 6.5ms의 수신 대기 시간

 // 응답 처리
  int bytes_read = 0;
  bool success = false;
 
 // RS485 채널에 따라 데이터 읽기
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf1[i] =  modbus_buf1[i+1];
          }
        
          uint8_t receivedByte = Serial1.read();
          modbus_buf1[NO_DATA-1] = receivedByte;    
          bytes_read++;             
      }
    
      if(bytes_read > 0) 
      {
        success = read_modbus_enable_handler_485(modbus_buf1);
        //Serial.print("Success 1:");
        //Serial.println(success);       
      }
      break;
    
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {

          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf2[i] =  modbus_buf2[i+1];
          }
          uint8_t receivedByte = Serial2.read();
          modbus_buf2[NO_DATA-1] = receivedByte;   
          bytes_read++;               
      }
      
      if(bytes_read > 0) {
        success = read_modbus_enable_handler_485(modbus_buf2);
      }
      break;
    
    case 3:
    while(Serial3.available() && bytes_read < 15) 
    {

        for(int i=0; i < NO_DATA; i++) 
        {
          modbus_buf3[i] =  modbus_buf3[i+1];
        }
        uint8_t receivedByte = Serial3.read();
        modbus_buf3[NO_DATA-1] = receivedByte;   
        bytes_read++;               

    }
      
      if(bytes_read > 0) 
      {
        success = read_modbus_enable_handler_485(modbus_buf3);
      }
      break;
  } 



  
  // 디버그 출력 (필요 시)
  if(DEBUG_ENABLE == true)
  {
    Serial.print("Sent Modbus Enable command to motor ID: 0x");
    Serial.print(motor_id, HEX);
    Serial.print(" - Full command: ");
    
    for (int i = 0; i < sizeof(full_cmd); i++) 
    {
      printHex(full_cmd[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

}


void sendDriverOutputEnable(uint8_t RS485_channel, uint8_t motor_id, bool enable) 
{
  command_type = 1;
  // 명령 프레임 구성: [ID + Function Code + Register Address + Data]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x06,       // Function Code (Write Single Register)
    0x00, 0x01, // Register Address (Driver Output Enable: 0x0001)
    0x00, static_cast<uint8_t>(enable ? 0x01 : 0x00)  // Data (Enable: 0x0001, Disable: 0x0000)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }

  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(6500);  // 6.5ms의 수신 대기 시간
  
  // 응답 처리
  int bytes_read = 0;
  bool success = false;
 
  // RS485 채널에 따라 데이터 읽기
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf1[i] = modbus_buf1[i+1];
          }
        
          uint8_t receivedByte = Serial1.read();
          modbus_buf1[NO_DATA-1] = receivedByte;    
          bytes_read++;             
      }
      
      if(bytes_read > 0) 
      {
        success = read_drive_output_enable_handler_485(modbus_buf1);
        //Serial.print("Drive Output Enable Success 1:");
        //Serial.println(success);
        
      }
      break;
    
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf2[i] = modbus_buf2[i+1];
          }
          uint8_t receivedByte = Serial2.read();
          modbus_buf2[NO_DATA-1] = receivedByte;   
          bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        success = read_drive_output_enable_handler_485(modbus_buf2);
        if(DEBUG_ENABLE) 
        {
          Serial.print("Drive Output Enable Success 2:");
          Serial.println(success);
        }
      }
      break;
    
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf3[i] = modbus_buf3[i+1];
          }
          uint8_t receivedByte = Serial3.read();
          modbus_buf3[NO_DATA-1] = receivedByte;   
          bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        success = read_drive_output_enable_handler_485(modbus_buf3);
        if(DEBUG_ENABLE) 
        {
          Serial.print("Drive Output Enable Success 3:");
          Serial.println(success);
        }
      }
      break;
  }
  
  // 디버그 출력 (필요 시)
  if(DEBUG_ENABLE == true)
  {
    Serial.print("Sent Driver Output ");
    Serial.print(enable ? "Enable" : "Disable");
    Serial.print(" command to motor ID: 0x");
    Serial.print(motor_id, HEX);
    Serial.print(" - Full command: ");
    
    for (int i = 0; i < sizeof(full_cmd); i++) 
    {
      printHex(full_cmd[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    if(bytes_read == 0)
    {
      Serial.println("No response received");
    }
  }
}

int32_t sendReadEncoder(uint8_t RS485_channel, uint8_t motor_id) 
{
  // 명령 프레임 구성: [ID + Function Code + Register Address + Number of Registers]
  command_type = 0x16;
  uint8_t cmd[] = 
  {
      motor_id,    // Device ID
      0x03,       // Function Code (Read Holding Registers)
      0x00,       // Register Address High byte
      0x16,       // Register Address Low byte (엔코더 레지스터 주소)
      0x00,       // Number of registers High byte
      0x02        // Number of registers Low byte (4바이트 데이터이므로 2개 레지스터)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송을 위한 전체 프레임 준비
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;         // CRC Low byte
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;  // CRC High byte
  
  // RS485로 명령 전송
  switch(RS485_channel)
  {
      case 1: 
          RS485_Send1(full_cmd, sizeof(full_cmd));
          break;
      case 2: 
          RS485_Send2(full_cmd, sizeof(full_cmd));
          break;         
      case 3: 
          RS485_Send3(full_cmd, sizeof(full_cmd));
          break; 
  }
  
  // 디버그 출력 
  if(DEBUG_ENABLE)
  {
      Serial.print("Send Encoder Read command - Motor ID: 0x");
      Serial.print(motor_id, HEX);
      Serial.print(" - Full command: ");
      
      for(int i = 0; i < sizeof(full_cmd); i++)
      {
          printHex(full_cmd[i]);
          Serial.print(" ");
      }
      Serial.println();
  }
  
  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(7000);  // 6.5ms의 대기 시간
  
  // 응답 처리
  int bytes_read = 0;
  bool success = false;
  int32_t encoder_value = 0;
  
  // RS485 채널에 따라 데이터 읽기 및 처리
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf1[i] = modbus_buf1[i+1];
          }
        
          uint8_t receivedByte = Serial1.read();
          modbus_buf1[NO_DATA-1] = receivedByte;    
          bytes_read++;             
      }
      
      if(bytes_read > 0) 
      {
        success = read_encoder_handler_485_1(modbus_buf1, &encoder_value);
        if(success && DEBUG_ENABLE) 
        {
          Serial.print("Encoder value from motor ");
          Serial.print(motor_id);
          Serial.print(": ");
          Serial.println(encoder_value);
        }
      }
      break;
    
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf2[i] = modbus_buf2[i+1];
          }
          uint8_t receivedByte = Serial2.read();
          modbus_buf2[NO_DATA-1] = receivedByte;   
          bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        success = read_encoder_handler_485_2(modbus_buf2, &encoder_value);
        if(success && DEBUG_ENABLE) 
        {
          Serial.print("Encoder value from motor ");
          Serial.print(motor_id);
          Serial.print(": ");
          Serial.println(encoder_value);
        }
      }
      break;
    
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
          for(int i=0; i < NO_DATA-1; i++) 
          {
            modbus_buf3[i] = modbus_buf3[i+1];
          }
          uint8_t receivedByte = Serial3.read();
          modbus_buf3[NO_DATA-1] = receivedByte;   
          bytes_read++;               
      }
      
      if(bytes_read > 0 && modbus_buf3 != NULL) 
      {
        // 만약 channel 3용 핸들러가 있다면 사용
        success = read_encoder_handler_485_3(modbus_buf3, &encoder_value);
        if(success && DEBUG_ENABLE) 
        {
          Serial.print("Encoder value from motor ");
          Serial.print(motor_id);
          Serial.print(": ");
          Serial.println(encoder_value);
        }
      }
      break;
  }
  
  // 디버그 출력 - 응답 없음 또는 실패 시
  if(DEBUG_ENABLE && !success) 
  {
    if(bytes_read == 0) 
    {
      Serial.print("No encoder response received from motor ");
      Serial.println(motor_id);
    } 
    else 
    {
      Serial.print("Invalid encoder response or CRC check failed for motor ");
      Serial.println(motor_id);
    }
  }

  return encoder_value;
}

/**
 * Modbus Enable 상태를 확인하는 함수
 * @param RS485_channel 사용할 RS485 채널 (1-3)
 * @param motor_id 모터 ID
 * @return Modbus가 활성화되었으면 true, 아니면 false
 */
bool checkModbusEnableStatus(uint8_t RS485_channel, uint8_t motor_id) 
{
  command_type = 0x20;  // Unique command type for modbus enable status check
  
  // Command frame: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = {
    motor_id,    // Device ID
    0x03,       // Function Code (Read Holding Registers)
    0x00, 0x00, // Register Address (ModbusEnable: 0x0000)
    0x00, 0x01  // Number of registers to read (1 register)
  };
  
  // Calculate CRC
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  // Send command via appropriate RS485 channel
  switch(RS485_channel) 
  {
    case 1: 
      RS485_Send1(full_cmd, sizeof(full_cmd));
      break;
    case 2: 
      RS485_Send2(full_cmd, sizeof(full_cmd));
      break;         
    case 3: 
      RS485_Send3(full_cmd, sizeof(full_cmd));
      break; 
    default:
      return false;
  }
  
  // Wait for response
  delayMicroseconds(6500);
  
  // Process response
  int bytes_read = 0;
  bool status = false;
  bool success = false;
  
  // Read from appropriate channel using the correct buffer
  switch(RS485_channel) 
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf1[i] = modbus_buf1[i+1];
        }
        uint8_t receivedByte = Serial1.read();
        modbus_buf1[NO_DATA-1] = receivedByte;
        bytes_read++;        
        
      }
      
      if(bytes_read > 0) {
         
        success = handleModbusEnableStatusResponse(modbus_buf1, motor_id, &status);
      }
      break;
      
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf2[i] = modbus_buf2[i+1];
        }
        uint8_t receivedByte = Serial2.read();
        modbus_buf2[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) {
        success = handleModbusEnableStatusResponse(modbus_buf2, motor_id, &status);
      }
      break;
      
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf3[i] = modbus_buf3[i+1];
        }
        uint8_t receivedByte = Serial3.read();
        modbus_buf3[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) {
        success = handleModbusEnableStatusResponse(modbus_buf3, motor_id, &status);
      }
      break;
  }
  
  if(DEBUG_ENABLE && !success) 
  {
    Serial.print("Failed to get Modbus Enable Status for Motor ID ");
    Serial.println(motor_id);
  }
  
  return success ? status : false;
}


bool checkDriverOutputStatus(uint8_t RS485_channel, uint8_t motor_id) 
{
  command_type = 0x21;  // Unique command type for driver output status check
  
  // Command frame: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = {
    motor_id,    // Device ID
    0x03,       // Function Code (Read Holding Registers)
    0x00, 0x01, // Register Address (Driver Output Enable: 0x0001)
    0x00, 0x01  // Number of registers to read (1 register)
  };
  
  // Calculate CRC
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  // Send command via appropriate RS485 channel
  switch(RS485_channel) {
    case 1: 
      RS485_Send1(full_cmd, sizeof(full_cmd));
      break;
    case 2: 
      RS485_Send2(full_cmd, sizeof(full_cmd));
      break;         
    case 3: 
      RS485_Send3(full_cmd, sizeof(full_cmd));
      break; 
    default:
      return false;
  }
  
  // Wait for response
  delayMicroseconds(6500);  // 6.5ms wait time for response

  // Process response
  int bytes_read = 0;
  bool status = false;
  bool success = false;
  
  // Read from appropriate channel and process response
  switch(RS485_channel) 
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf1[i] = modbus_buf1[i+1];
        }
        uint8_t receivedByte = Serial1.read();
        modbus_buf1[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) 
      {
        success = handleDriverOutputStatusResponse(modbus_buf1, motor_id, &status);
      }
      break;
      
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf2[i] = modbus_buf2[i+1];
        }
        uint8_t receivedByte = Serial2.read();
        modbus_buf2[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) 
      {
        success = handleDriverOutputStatusResponse(modbus_buf2, motor_id, &status);
      }
      break;
      
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf3[i] = modbus_buf3[i+1];
        }
        uint8_t receivedByte = Serial3.read();
        modbus_buf3[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) 
      {
        success = handleDriverOutputStatusResponse(modbus_buf3, motor_id, &status);
      }
      break;
  }
  
  if(DEBUG_ENABLE && !success) 
  {
    Serial.print("Failed to get Driver Output Status for Motor ID ");
    Serial.println(motor_id);
  }
  
  return success ? status : false;
   
}

/**
 * 특정 모터의 상태를 확인하고 필요한 경우 활성화하는 함수
 * @param RS485_channel 사용할 RS485 채널 (1-3)
 * @param motor_id 모터 ID
 * @param motorType 모터 유형 (설명용 문자열)
 * @return 성공적으로 활성화되었으면 true, 실패했으면 false
 */
bool checkAndEnableMotor(uint8_t RS485_channel, uint8_t motor_id, const char* motorType) 
{
  const uint16_t delayBetweenChecks = 30;  // 검사 사이 지연 시간 (ms)
  bool modbusEnabled = false;
  bool driverEnabled = false;
  if(DEBUG_ENABLE)
  {
    Serial.print(motorType);
    Serial.print(" ID ");
    Serial.print(motor_id);
    Serial.println(": Checking status...");
  }

  // Modbus Enable 확인 및 활성화
  modbusEnabled = checkAndEnableModbus(RS485_channel, motor_id);
  delay(delayBetweenChecks);
  
  if (!modbusEnabled) 
  {
    if(DEBUG_ENABLE)
    {
      Serial.print(motorType);
      Serial.print(" ID ");
      Serial.print(motor_id);
      Serial.println(": Failed to enable Modbus. Skipping driver output check.");
    }
    return false;
  }
  
  // Driver Output Enable 확인 및 활성화
  driverEnabled = checkAndEnableDriverOutput(RS485_channel, motor_id);
  
  return driverEnabled;
}

/**
 * 모든 모터의 출력 상태를 확인하고, 필요한 경우 활성화하는 함수
 * RS485 채널 별로 연결된 모터들을 확인하고 처리합니다.
 */
void checkAndEnableAllMotors() 
{
  const uint16_t delayBetweenMotors = 50;  // 모터 사이 지연 시간 (ms)
  
  Serial.println("Checking and enabling all motors if needed...");
  
  // 채널 1의 휠 모터 확인 (ID: 1-4)
  for (uint8_t motorId = 1; motorId <= 4; motorId++) 
  {
    checkAndEnableMotor(1, motorId, "Wheel Motor");
    delay(delayBetweenMotors);
  }
  
  // 채널 2의 스티어링 모터 확인 (ID: 1-2)
  for (uint8_t motorId = 1; motorId <= 2; motorId++) 
  {
    checkAndEnableMotor(2, motorId, "Steering Motor");
    delay(delayBetweenMotors);
  }
  
  //Serial.println("Motor status check and enable process completed");
}



// Example usage
void checkAllMotorsStatus() 
{
  // Check front wheel motors (assuming they're on RS485 channel 1)
  bool frontLeftEnabled = checkDriverOutputStatus(1, 1);
  bool frontRightEnabled = checkDriverOutputStatus(1, 2);
  
  // Check rear wheel motors
  bool rearLeftEnabled = checkDriverOutputStatus(1, 3);
  bool rearRightEnabled = checkDriverOutputStatus(1, 4);
  
  // Check steering motors (assuming they're on RS485 channel 2)
  bool frontSteeringEnabled = checkDriverOutputStatus(2, 1);
  bool rearSteeringEnabled = checkDriverOutputStatus(2, 2);
  
  // Print status to serial monitor
  if(DEBUG_ENABLE)
  {
    Serial.println("Motor Driver Output Status:");
    Serial.print("Front Left: "); Serial.println(frontLeftEnabled ? "Enabled" : "Disabled");
    Serial.print("Front Right: "); Serial.println(frontRightEnabled ? "Enabled" : "Disabled");
    Serial.print("Rear Left: "); Serial.println(rearLeftEnabled ? "Enabled" : "Disabled");
    Serial.print("Rear Right: "); Serial.println(rearRightEnabled ? "Enabled" : "Disabled");
    Serial.print("Front Steering: "); Serial.println(frontSteeringEnabled ? "Enabled" : "Disabled");
    Serial.print("Rear Steering: "); Serial.println(rearSteeringEnabled ? "Enabled" : "Disabled");
  }
  
}



/**
 * 특정 모터의 Modbus Enable 상태를 확인하고 필요시 활성화하는 함수
 * @param RS485_channel 사용할 RS485 채널 (1-3)
 * @param motor_id 모터 ID
 * @return 성공적으로 활성화되었으면 true, 실패했으면 false
 */
bool checkAndEnableModbus(uint8_t RS485_channel, uint8_t motor_id) 
{
  const uint16_t delayAfterCommand = 70;  // 명령 후 지연 시간 (ms)
  bool isEnabled = checkModbusEnableStatus(RS485_channel, motor_id);
  
  if (!isEnabled) 
  {
    if (DEBUG_ENABLE) 
    {
      Serial.print("Motor ID ");
      Serial.print(motor_id);
      Serial.print(" on RS485 channel ");
      Serial.print(RS485_channel);
      Serial.println(": Modbus is disabled. Enabling...");
    }  
    // Modbus Enable 활성화
    sendModbusEnable(RS485_channel, motor_id, true);
    delay(delayAfterCommand);
    
    // 활성화 후 상태 다시 확인
    isEnabled = checkModbusEnableStatus(RS485_channel, motor_id);
    
    if (DEBUG_ENABLE) 
    {
      Serial.print("Motor ID ");
      Serial.print(motor_id);
      Serial.print(" on RS485 channel ");
      Serial.print(RS485_channel);
      Serial.print(": Modbus is now ");
      Serial.println(isEnabled ? "Enabled" : "Still Disabled");
    }
  } 
  else 
  {
    if (DEBUG_ENABLE) 
    {
      Serial.print("Motor ID ");
      Serial.print(motor_id);
      Serial.print(" on RS485 channel ");
      Serial.print(RS485_channel);
      Serial.println(": Modbus is already enabled");
    }
  }
  
  return isEnabled;
}


/**
 * 특정 모터의 Driver Output 상태를 확인하고 필요시 활성화하는 함수
 * @param RS485_channel 사용할 RS485 채널 (1-3)
 * @param motor_id 모터 ID
 * @return 성공적으로 활성화되었으면 true, 실패했으면 false
 */
bool checkAndEnableDriverOutput(uint8_t RS485_channel, uint8_t motor_id) 
{
  const uint16_t delayAfterCommand = 70;  // 명령 후 지연 시간 (ms)
  bool isEnabled = checkDriverOutputStatus(RS485_channel, motor_id);
  
  if (!isEnabled) 
  {
    if (DEBUG_ENABLE) 
    {
      Serial.print("Motor ID ");
      Serial.print(motor_id);
      Serial.print(" on RS485 channel ");
      Serial.print(RS485_channel);
      Serial.println(": Driver Output is disabled. Enabling...");
    }
    // Driver Output Enable 활성화
    sendDriverOutputEnable(RS485_channel, motor_id, true);
    delay(delayAfterCommand);
    
    // 활성화 후 상태 다시 확인
    isEnabled = checkDriverOutputStatus(RS485_channel, motor_id);
    
    if (DEBUG_ENABLE) 
    {
      Serial.print("Motor ID ");
      Serial.print(motor_id);
      Serial.print(" on RS485 channel ");
      Serial.print(RS485_channel);
      Serial.print(": Driver Output is now ");
      Serial.println(isEnabled ? "Enabled" : "Still Disabled");
    }
  } 
  else 
  {
    if (DEBUG_ENABLE) 
    {
      Serial.print("Motor ID ");
      Serial.print(motor_id);
      Serial.print(" on RS485 channel ");
      Serial.print(RS485_channel);
      Serial.println(": Driver Output is already enabled");
    }
  }
  
  return isEnabled;
}





void send_steering_encoder_status(uint8_t motor_id)
{
  uint8_t encoder_steering[4];

  //encoder_pos_steering[0] = 100;
  //encoder_pos_steering[1] = -200;

  motor_id = (motor_id <=1) ? motor_id = 1 : motor_id;
  motor_id = (motor_id >=2) ? motor_id = 2 : motor_id;


  encoder_steering[0] = (uint8_t)(encoder_pos_steering[motor_id-1] & 0xFF);         // 최하위 바이트
  encoder_steering[1] = (uint8_t)((encoder_pos_steering[motor_id-1] >> 8) & 0xFF);  // 하위 중간 바이트
  encoder_steering[2] = (uint8_t)((encoder_pos_steering[motor_id-1] >> 16) & 0xFF); // 상위 중간 바이트
  encoder_steering[3] = (uint8_t)((encoder_pos_steering[motor_id-1] >> 24) & 0xFF); // 최상위 바이트


  uint8_t cmd[] = 
  {
    '#',                // Start character 0x23
    'R',                // Command type    0x52
    'B',                // Command         0x45
    motor_id,
    encoder_steering[0],               // 4byte motor
    encoder_steering[1],               // Reserved , ID   0x00
    encoder_steering[2],               // Reserved , ID   0x00
    encoder_steering[3]                // Reserved , ID   0x00    
  };
  // CRC 계산 - '*' 문자를 제외한 앞부분만 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 전체 명령 준비
  uint8_t full_cmd[11];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[8] = crc & 0xFF;          // CRC Low byte
  full_cmd[9] = (crc >> 8) & 0xFF;   // CRC High byte
  full_cmd[10] = '*';                // End character 0x2A
  
  // 시리얼로 전송
  Serial.write(full_cmd, sizeof(full_cmd));
}


void send_wheel_encoder_status(uint8_t motor_id)
{

  uint8_t encoder_front_left[4];
  //엔코더 값 추출 (4바이트, Little Endian)
  /*  
  encoder_pos_wheel[0] =  100;
  encoder_pos_wheel[1] = -200;
  encoder_pos_wheel[2] =  300;
  encoder_pos_wheel[3] = -400;
  */


  motor_id = (motor_id <=1) ? motor_id = 1 : motor_id;
  motor_id = (motor_id >=4) ? motor_id = 4 : motor_id;

  encoder_front_left[0] = (uint8_t)(encoder_pos_wheel[motor_id-1] & 0xFF);         // 최하위 바이트
  encoder_front_left[1] = (uint8_t)((encoder_pos_wheel[motor_id-1] >> 8) & 0xFF);  // 하위 중간 바이트
  encoder_front_left[2] = (uint8_t)((encoder_pos_wheel[motor_id-1] >> 16) & 0xFF); // 상위 중간 바이트
  encoder_front_left[3] = (uint8_t)((encoder_pos_wheel[motor_id-1] >> 24) & 0xFF); // 최상위 바이트

  uint8_t cmd[] = 
  {
    '#',                // Start character 0x23
    'R',                // Command type    0x52
    'E',                // Command         0x45
    motor_id,
    encoder_front_left[0],               // 4byte motor
    encoder_front_left[1],               // Reserved , ID   0x00
    encoder_front_left[2],               // Reserved , ID   0x00
    encoder_front_left[3]                // Reserved , ID   0x00    
  };
  
  // CRC 계산 - '*' 문자를 제외한 앞부분만 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 전체 명령 준비
  uint8_t full_cmd[11];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[8] = crc & 0xFF;          // CRC Low byte
  full_cmd[9] = (crc >> 8) & 0xFF;   // CRC High byte
  full_cmd[10] = '*';                // End character 0x2A
  
  // 시리얼로 전송
  Serial.write(full_cmd, sizeof(full_cmd));

}


void send_Limit_Switch_Status(void)
{
  uint8_t fork_limit_switch_byte = 0;
  uint8_t steering_limit_switch_byte = 0;

  fork_limit_switch_status[0] = digitalRead(D26);  //LSB
  fork_limit_switch_status[1] = digitalRead(D27);
  fork_limit_switch_status[2] = digitalRead(D42);
  fork_limit_switch_status[3] = digitalRead(D43);
  fork_limit_switch_status[4] = digitalRead(D44);
  fork_limit_switch_status[5] = digitalRead(D45);
  fork_limit_switch_status[6] = digitalRead(D46);
  fork_limit_switch_status[7] = digitalRead(D47);  // MSB

  front_steering_limit_switch_status[0]  = digitalRead(D22);
    
  front_steering_limit_switch_status[1]  = digitalRead(D23);
  
  rear_steering_limit_switch_status[0]   = digitalRead(D24);
  rear_steering_limit_switch_status[1]   = digitalRead(D25);
  front_steering_limit_switch_status[0]  = LOW;
  front_steering_limit_switch_status[1]  = LOW;
  rear_steering_limit_switch_status[0]   = LOW;
  rear_steering_limit_switch_status[1]  = HIGH;
  //Protocol '#' + 'R' + 'L' + 0x00 + 0x00 + Limit Switch Status + CRC16 +'*'
  //         0x23  0x52  0x4C          
  // 8개의 스위치 상태를 1바이트로 조합
  fork_limit_switch_byte = fork_limit_switch_status[7] << 7 | 
                      fork_limit_switch_status[6] << 6 |
                      fork_limit_switch_status[5] << 5 |
                      fork_limit_switch_status[4] << 4 |
                      fork_limit_switch_status[3] << 3 |
                      fork_limit_switch_status[2] << 2 |
                      fork_limit_switch_status[1] << 1 |
                      fork_limit_switch_status[0];

  steering_limit_switch_byte  =  front_steering_limit_switch_status[0] |
                                 front_steering_limit_switch_status[1] << 1 |
                                 rear_steering_limit_switch_status[0]  << 2 |
                                 rear_steering_limit_switch_status[1]  << 3 ;
  uint8_t cmd[] = 
  {
    '#',                // Start character 0x23
    'R',                // Command type    0x52
    'L',                // Command         0x4c 
    0x00,               // Reserved , ID   0x00
    0x00,               // Reserved        0x00
    steering_limit_switch_byte,  // Limit switch status
    fork_limit_switch_byte,  // Limit switch status
    0x00                // Reserved
  };
  
  // CRC 계산 - '*' 문자를 제외한 앞부분만 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 전체 명령 준비
  uint8_t full_cmd[11];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[8] = crc & 0xFF;          // CRC Low byte
  full_cmd[9] = (crc >> 8) & 0xFF;   // CRC High byte
  full_cmd[10] = '*';                // End character
  
  // 시리얼로 전송
  Serial.write(full_cmd, sizeof(full_cmd));
}

// limit switch steering
bool read_limit_switch_front_steering_left(void)
{
   return digitalRead(D22);  //LSB
}

bool read_limit_switch_front_steering_right(void)
{
   return digitalRead(D23);  //LSB
}

bool read_limit_switch_rear_steering_left(void)
{
   return digitalRead(D24);  //LSB  
}

bool read_limit_switch_rear_steering_right(void)
{
   return digitalRead(D25);  //LSB
}

// fork limit switch
bool read_limit_switch_fork_left_front_open(void)
{
   return digitalRead(D26);  //LSB
}

bool read_limit_switch_fork_left_front_close(void)
{
   return digitalRead(D27);  //LSB
}

bool read_limit_switch_fork_right_front_open(void)
{
   return digitalRead(D42);  //LSB
}

bool read_limit_switch_fork_right_front_close(void)
{
   return digitalRead(D43);  //LSB
}

bool read_limit_switch_fork_left_rear_open(void)
{
   return digitalRead(D44);  //LSB
}

bool read_limit_switch_fork_left_rear_close(void)
{
   return digitalRead(D45);  //LSB
}

bool read_limit_switch_fork_right_rear_open(void)
{
   return digitalRead(D46);  //LSB
}

bool read_limit_switch_fork_right_rear_close(void)
{
   return digitalRead(D47);  //LSB
}

bool handleModbusEnableStatusResponse(uint8_t* buffer, uint8_t motor_id, bool* status) 
{
  // Check for valid response pattern
  for(int i = NO_DATA-6; i >= 0; i--) 
  {
    if((buffer[i] == motor_id) && 
       (buffer[i+1] == 0x03) && 
       (buffer[i+2] == 0x02)) {  // Data length is 2 bytes
      
      uint16_t receivedCRC = (buffer[i+6] << 8) | buffer[i+5];
      uint16_t calculatedCRC = CRC16_MODBUS(&buffer[i], 5);
      
      //Serial.println(receivedCRC);
      //Serial.println(calculatedCRC);
      
      if(receivedCRC == calculatedCRC) 
      {
        // Extract modbus enable status from response
        *status = (buffer[i+4] & 0x01) == 0x01;
     
        if(DEBUG_ENABLE) 
        {
          Serial.print("Modbus Enable Status for Motor ID ");
          Serial.print(motor_id);
          Serial.print(": ");
          Serial.println(*status ? "Enabled" : "Disabled");
        }        
        return true;
      }
    }
  }
  
  return false;
}

bool handleDriverOutputStatusResponse(uint8_t* buffer, uint8_t motor_id, bool* status) 
{
  // Check for valid response pattern
  for(int i = NO_DATA-6; i >= 0; i--) 
  {
    if((buffer[i] == motor_id) && 
       (buffer[i+1] == 0x03) && 
       (buffer[i+2] == 0x02)) 
       {  // Data length is 2 bytes
      
      uint16_t receivedCRC = (buffer[i+6] << 8) | buffer[i+5];
      uint16_t calculatedCRC = CRC16_MODBUS(&buffer[i], 5);
      
      if(receivedCRC == calculatedCRC) 
      {
        // Extract driver output enable status from response
        *status = (buffer[i+4] & 0x01) == 0x01;
        
        if(DEBUG_ENABLE) 
        {
          Serial.print("Driver Output Status for Motor ID ");
          Serial.print(motor_id);
          Serial.print(": ");
          Serial.println(*status ? "Enabled" : "Disabled");
        }
        
        return true;
      }
    }
  }
  
  return false;
}



// 응답 처리 함수1  
bool handleDriverOutputResponse1(uint8_t RS485_channel,uint8_t dataLength) 
{
  if (dataLength < 8) return false;  // 최소 길이 체크 (응답 6바이트 + CRC 2바이트)
  
  // 응답 프레임 확인
  if (modbus_buf1[1] == 0x06 && // Function Code
      modbus_buf1[2] == 0x00 && // Register Address High
      modbus_buf1[3] == 0x01) { // Register Address Low
    
    // CRC 검증
    uint16_t receivedCRC = (modbus_buf1[7] << 8) | modbus_buf1[6];
    uint16_t calculatedCRC = CRC16_MODBUS(modbus_buf1, 6);
    
    if (receivedCRC == calculatedCRC) 
    {
      // 명령 상태 출력
      bool isEnabled = (modbus_buf1[5] == 0x01);
      Serial.print("Driver Output ");
      Serial.print(isEnabled ? "Enable" : "Disable");
      Serial.println(" command successful");
      return true;
    }
  }  
  Serial.println("Invalid Driver Output response");
  return false;
}

// 응답 처리 함수  
bool handleDriverOutputResponse2(uint8_t RS485_channel,uint8_t dataLength) 
{
  if (dataLength < 8) return false;  // 최소 길이 체크 (응답 6바이트 + CRC 2바이트)
  
  // 응답 프레임 확인
  if (modbus_buf2[1] == 0x06 && // Function Code
      modbus_buf2[2] == 0x00 && // Register Address High
      modbus_buf2[3] == 0x01) { // Register Address Low
    
    // CRC 검증
    uint16_t receivedCRC = (modbus_buf2[7] << 8) | modbus_buf2[6];
    uint16_t calculatedCRC = CRC16_MODBUS(modbus_buf2, 6);
    
    if (receivedCRC == calculatedCRC) 
    {
      // 명령 상태 출력
      bool isEnabled = (modbus_buf2[5] == 0x01);
      Serial.print("Driver Output ");
      Serial.print(isEnabled ? "Enable" : "Disable");
      Serial.println(" command successful");
      return true;
    }
  }  
  Serial.println("Invalid Driver Output response");
  return false;
}

// 타겟 속도 읽기 응답 처리 함수
bool handleReadTargetSpeedResponse(uint8_t* data, uint8_t motor_id, int16_t* target_speed)
{
  /*
  for(int i =0; i< NO_DATA ; i++)
  {
    Serial.print(data[i],HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  */
  for(int i = NO_DATA-7; i >= 0; i--)
  {
    if((data[i] == motor_id) && 
       (data[i+1] == 0x03) && 
       (data[i+2] == 0x02))  // 데이터 길이 2바이트
    {
      uint16_t receivedCRC = (data[i+6] << 8) | data[i+5];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 5);
      //Serial.println(receivedCRC);
      //Serial.println(calculatedCRC);

      if(receivedCRC == calculatedCRC) 
      {
        // 응답 데이터에서 속도 값 추출
        *target_speed = (data[i+3] << 8) | data[i+4];
        
        // 음수 처리 (2의 보수 형태로 표현된 경우)
        if(*target_speed > 32767) {
          *target_speed -= 65536;
        }
        
        if(DEBUG_ENABLE) 
        {
          Serial.print("Read Target Speed Response - ID: 0x");
          Serial.print(motor_id, HEX);
          Serial.print(" Current Target Speed: ");
          Serial.print(*target_speed);
          Serial.println(" r/min");
        }
        
        // 버퍼 초기화
        memset(data, 0, NO_DATA);
        return true;
      }
    }
  }
  
  return false;
}


// 타겟 위치 이동 명령 응답 처리 핸들러
bool handle_move_to_position_response(uint8_t* response_buffer, int bytes_read, uint8_t motor_id, long* response_position)
{
    if(bytes_read < 8) // 최소 응답 길이 확인
        return false;
    
    // 모터 ID와 함수 코드 확인
    if(response_buffer[0] == motor_id && response_buffer[1] == 0x7B)
    {
        // CRC 계산 및 검증
        uint16_t receivedCRC = (response_buffer[bytes_read-1] << 8) | response_buffer[bytes_read-2];
        uint16_t calculatedCRC = CRC16_MODBUS(response_buffer, bytes_read - 2);
        
        if(receivedCRC == calculatedCRC)
        {
            // 응답에서 위치 값을 추출 (필요한 경우)
            // 응답 포맷: [motor_id][0x7B][position_data (4 bytes)][CRC (2 bytes)]
            if(bytes_read >= 6 && response_position != NULL) // 4바이트 위치 데이터 + ID + 함수코드
            {
                *response_position = (((uint32_t)response_buffer[2] << 24) | 
                                     ((uint32_t)response_buffer[3] << 16) | 
                                     ((uint32_t)response_buffer[4] << 8) | 
                                     ((uint32_t)response_buffer[5]));
            }
            
            return true;
        }
    }
    
    return false;
}

// Motor target speed 설정 함수
void setMotorTargetSpeed(uint8_t RS485_channel, uint8_t motor_id, int16_t speed)
{
  command_type = 2;  // Motor target speed 명령 타입
  
  // 속도 범위 제한 (-2000~2000 r/min)
  if(speed > 2000) 
  {
    speed = 2000;
    Serial.println("Warning: Speed limited to +2000 r/min");
  }
  else if(speed < -2000)
  {
    speed = -2000;
    Serial.println("Warning: Speed limited to -2000 r/min");
  }
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Data]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x06,       // Function Code (Write Single Register)
    0x00, 0x02, // Register Address (Motor target speed: 0x0002)
    (uint8_t)(speed >> 8),    // Speed High Byte
    (uint8_t)(speed & 0xFF)   // Speed Low Byte
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 전체 명령 버퍼 준비 (명령 + CRC)
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
    
  // RS485로 명령 전송
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
  
  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(6500);  // 6.5ms의 수신 대기 시간
  
  // 응답 처리
  int bytes_read = 0;
  bool success = false;
  int16_t response_speed = 0;
 
  // RS485 채널에 따라 데이터 읽기 및 처리
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf1[i] = modbus_buf1[i+1];
        }
        
        uint8_t receivedByte = Serial1.read();
        modbus_buf1[NO_DATA-1] = receivedByte;    
        bytes_read++;             
      }
      
      if(bytes_read > 0) 
      {
        // 수신된 속도 명령 응답 분석
        for(int i = NO_DATA-7; i >= 0; i--)  
        {  
          if((modbus_buf1[i] == motor_id) && 
             (modbus_buf1[i+1] == 0x06) && 
             (modbus_buf1[i+2] == 0x00) && 
             (modbus_buf1[i+3] == 0x02)) 
          {
            uint16_t receivedCRC = (modbus_buf1[i+7] << 8) | modbus_buf1[i+6];
            uint16_t calculatedCRC = CRC16_MODBUS(&modbus_buf1[i], 6);
            
            if(receivedCRC == calculatedCRC) 
            {
              // 응답 데이터에서 속도 값 추출
              response_speed = (modbus_buf1[i+4] << 8) | modbus_buf1[i+5];
              // 음수 처리
              if(response_speed > 32767) {
                response_speed -= 65536;
              }
              success = true;
              
              if(DEBUG_ENABLE) 
              {
                Serial.print("Motor Target Speed Response - ID: 0x");
                Serial.print(motor_id, HEX);
                Serial.print(" Confirmed Speed: ");
                Serial.print(response_speed);
                Serial.println(" r/min");
              }
              
              // 버퍼 초기화
              memset(modbus_buf1, 0, NO_DATA);
              break;
            }
          }
        }
      }
      break;
    
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf2[i] = modbus_buf2[i+1];
        }
        uint8_t receivedByte = Serial2.read();
        modbus_buf2[NO_DATA-1] = receivedByte;   
        bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        // 수신된 속도 명령 응답 분석
        for(int i = NO_DATA-7; i >= 0; i--)  
        {  
          if((modbus_buf2[i] == motor_id) && 
             (modbus_buf2[i+1] == 0x06) && 
             (modbus_buf2[i+2] == 0x00) && 
             (modbus_buf2[i+3] == 0x02)) 
          {
            uint16_t receivedCRC = (modbus_buf2[i+7] << 8) | modbus_buf2[i+6];
            uint16_t calculatedCRC = CRC16_MODBUS(&modbus_buf2[i], 6);
            
            if(receivedCRC == calculatedCRC) 
            {
              // 응답 데이터에서 속도 값 추출
              response_speed = (modbus_buf2[i+4] << 8) | modbus_buf2[i+5];
              // 음수 처리
              if(response_speed > 32767) {
                response_speed -= 65536;
              }
              success = true;
              
              if(DEBUG_ENABLE) 
              {
                Serial.print("Motor Target Speed Response - ID: 0x");
                Serial.print(motor_id, HEX);
                Serial.print(" Confirmed Speed: ");
                Serial.print(response_speed);
                Serial.println(" r/min");
              }
              
              // 버퍼 초기화
              memset(modbus_buf2, 0, NO_DATA);
              break;
            }
          }
        }
      }
      break;
    
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf3[i] = modbus_buf3[i+1];
        }
        uint8_t receivedByte = Serial3.read();
        modbus_buf3[NO_DATA-1] = receivedByte;   
        bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        // 수신된 속도 명령 응답 분석
        for(int i = NO_DATA-7; i >= 0; i--)  
        {  
          if((modbus_buf3[i] == motor_id) && 
             (modbus_buf3[i+1] == 0x06) && 
             (modbus_buf3[i+2] == 0x00) && 
             (modbus_buf3[i+3] == 0x02)) 
          {
            uint16_t receivedCRC = (modbus_buf3[i+7] << 8) | modbus_buf3[i+6];
            uint16_t calculatedCRC = CRC16_MODBUS(&modbus_buf3[i], 6);
            
            if(receivedCRC == calculatedCRC) 
            {
              // 응답 데이터에서 속도 값 추출
              response_speed = (modbus_buf3[i+4] << 8) | modbus_buf3[i+5];
              // 음수 처리
              if(response_speed > 32767) {
                response_speed -= 65536;
              }
              success = true;
              
              if(DEBUG_ENABLE) 
              {
                Serial.print("Motor Target Speed Response - ID: 0x");
                Serial.print(motor_id, HEX);
                Serial.print(" Confirmed Speed: ");
                Serial.print(response_speed);
                Serial.println(" r/min");
              }
              
              // 버퍼 초기화
              memset(modbus_buf3, 0, NO_DATA);
              break;
            }
          }
        }
      }
      break;
  }
  
  // 디버그 출력
  if(DEBUG_ENABLE) 
  {
    Serial.print("Sent Motor Target Speed command to motor ID: 0x");
    Serial.print(motor_id, HEX);
    Serial.print(" Speed: ");
    Serial.print(speed);
    Serial.print(" r/min - Full command: ");
    
    // 전체 명령 (CRC 포함) 출력
    for (int i = 0; i < sizeof(cmd); i++) 
    {
      printHex(cmd[i]);
      Serial.print(" ");
    }
    printHex(crc & 0xFF);
    Serial.print(" ");
    printHex((crc >> 8) & 0xFF);
    Serial.println();
    
    if(bytes_read == 0) {
      Serial.println("No response received");
    }
    else if(!success) {
      Serial.println("Invalid response or CRC check failed");
    }
  }
}

// 모터 타겟 속도 읽기 함수
int16_t readTargetSpeed(uint8_t RS485_channel, uint8_t motor_id)
{
  command_type = 0x22;  // 모터 타겟 속도 읽기 명령 타입 (고유 식별자로 설정)
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x03,       // Function Code (Read Holding Registers)
    0x00, 0x02, // Register Address (Motor target speed: 0x0002)
    0x00, 0x01  // Number of registers to read (1 register = 2 bytes)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 전체 명령 버퍼 준비 (명령 + CRC)
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  // RS485로 명령 전송
  switch(RS485_channel)
  {
    case 1: 
      RS485_Send1(full_cmd, sizeof(full_cmd));
      break;
    case 2: 
      RS485_Send2(full_cmd, sizeof(full_cmd));
      break;         
    case 3: 
      RS485_Send3(full_cmd, sizeof(full_cmd));
      break; 
  }
  
  // 디버그 출력
  if(DEBUG_ENABLE)
  {
    Serial.print("Sent Read Target Speed command to motor ID: 0x");
    Serial.print(motor_id, HEX);
    Serial.print(" - Full command: ");
    
    for (int i = 0; i < sizeof(full_cmd); i++) 
    {
      printHex(full_cmd[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(6500);  // 6.5ms의 수신 대기 시간
  
  // 응답 처리
  int bytes_read = 0;
  int16_t target_speed = 0;
  bool success = false;

  // RS485 채널에 따라 데이터 읽기 및 처리
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf1[i] = modbus_buf1[i+1];
        }
        
        uint8_t receivedByte = Serial1.read();
        modbus_buf1[NO_DATA-1] = receivedByte;    
        bytes_read++;             
      }
      
      if(bytes_read > 0) 
      {
        success = handleReadTargetSpeedResponse(modbus_buf1, motor_id, &target_speed);
      }
      break;
    
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf2[i] = modbus_buf2[i+1];
        }
        uint8_t receivedByte = Serial2.read();
        modbus_buf2[NO_DATA-1] = receivedByte;   
        bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        success = handleReadTargetSpeedResponse(modbus_buf2, motor_id, &target_speed);
      }
      break;
    
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf3[i] = modbus_buf3[i+1];
        }
        uint8_t receivedByte = Serial3.read();
        modbus_buf3[NO_DATA-1] = receivedByte;   
        bytes_read++;               
      }
      
      if(bytes_read > 0) 
      {
        success = handleReadTargetSpeedResponse(modbus_buf3, motor_id, &target_speed);
      }
      break;
  }
  
  // 디버그 출력 - 응답 없음 또는 실패 시
  if(DEBUG_ENABLE && !success) 
  {
    if(bytes_read == 0) {
      Serial.println("No response received");
    } else {
      Serial.println("Invalid response or CRC check failed");
    }
  }
  
  return target_speed;
}





// Parameter save flag 설정 함수
void setParameterSaveFlag(uint8_t RS485_channel,uint8_t motor_id)
{
  command_type = 0x14;  // Parameter save flag command type
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Data]
  uint8_t cmd[] = 
  {
    motor_id,      // Device ID
    0x06,         // Function Code (Write Single Register)
    0x00, 0x14,   // Register Address (Parameter save flag: 0x0014)
    0x00, 0x01    // Data (1: saving)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;

  // RS485로 명령 전송
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
  
  // 디버그 출력
  if(DEBUG_ENABLE == false)  return;

  Serial.print("Sent Parameter Save Flag command to motor ID: 0x");
  Serial.print(motor_id, HEX);
  Serial.print(" - Full command: ");
  
  // 전체 명령 (CRC 포함) 출력
  for (int i = 0; i < sizeof(cmd); i++) 
  {
    printHex(cmd[i]);
    Serial.print(" ");
  }
  printHex(crc & 0xFF);
  Serial.print(" ");
  printHex((crc >> 8) & 0xFF);
  Serial.println();
}


void setEncoder(uint8_t RS485_channel, uint8_t motor_id, long encoderValue)
{
  setEncoder_Low(RS485_channel, motor_id, encoderValue);
  setEncoder_High(RS485_channel, motor_id, encoderValue);
}

void setEncoder_Low(uint8_t RS485_channel, uint8_t motor_id, long encoderValue) 
{
  // Command to set lower 16 bits of absolute position
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x06,        // Function Code (Write Single Register)
    0x00, 0x16,  // Register Address (Absolute position lower 16 bits: 0x0016)
    (uint8_t)((encoderValue & 0x0000FFFF) >> 8),    // Lower 16 bits high byte
    (uint8_t)(encoderValue & 0x00FF)                // Lower 16 bits low byte
  };
  
  // CRC calculation
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command with CRC
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;

  // Send via appropriate RS485 channel
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
}

void setEncoder_High(uint8_t RS485_channel, uint8_t motor_id, long encoderValue) 
{
  // Command to set higher 16 bits of absolute position
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x06,        // Function Code (Write Single Register)
    0x00, 0x17,  // Register Address (Absolute position high 16 bits: 0x0017)
    (uint8_t)((encoderValue & 0xFFFF0000) >> 24),   // Higher 16 bits high byte
    (uint8_t)((encoderValue & 0x00FF0000) >> 16)    // Higher 16 bits low byte
  };
  
  // CRC calculation
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command with CRC
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;

  // Send via appropriate RS485 channel
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
}

// Alarm code 읽기
void readAlarmCode(uint8_t RS485_channel, uint8_t motor_id)
{
  command_type = 0x0E;  // Alarm code command type
  
  // Command frame: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x03,       // Function Code (Read Holding Registers)
    0x00, 0x0E, // Register Address (Alarm code: 0x0E)
    0x00, 0x01  // Number of registers to read (1 register)
  };
  
  // CRC calculation
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command with CRC
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  // Send via appropriate RS485 channel
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
}


// Device Address 변경 함수
void setDeviceAddress(uint8_t RS485_channel,uint8_t current_motor_id, uint8_t new_address)
{
  command_type = 0x15;  // Device Address command type
  
  // 주소 범위 체크 (0~255)
  if(new_address > 255)
  {
    Serial.println("Warning: Address out of range (0-255)");
    return;
  }
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Data]
  uint8_t cmd[] = 
  {
    current_motor_id, // Current Device ID
    0x06,            // Function Code (Write Single Register)
    0x00, 0x15,      // Register Address (Device Address: 0x0015)
    0x00, new_address // New device address
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  RS485_Send1(full_cmd, sizeof(full_cmd));
  
  // 디버그 출력
  if(DEBUG_ENABLE == false)  return;

  Serial.print("Sent Device Address Change command - Current ID: 0x");
  Serial.print(current_motor_id, HEX);
  Serial.print(" New Address: 0x");
  Serial.print(new_address, HEX);
  Serial.print(" - Full command: ");
  
  // 전체 명령 (CRC 포함) 출력
  for (int i = 0; i < sizeof(cmd); i++) 
  {
    printHex(cmd[i]);
    Serial.print(" ");
  }
  printHex(crc & 0xFF);
  Serial.print(" ");
  printHex((crc >> 8) & 0xFF);
  Serial.println();
}

void move_relative_position(uint8_t RS485_channel, uint8_t motor_id, long relative_pos) 
{
    // 상대 위치 이동을 위한 명령어 프레임 구성 (Function code 0x78)
    uint8_t cmd[] = 
    {
        motor_id,                                    // 모터 ID
        0x78,                                       // Function code for relative position move
        (uint8_t)((relative_pos >> 24) & 0xFF),    // Position highest byte
        (uint8_t)((relative_pos >> 16) & 0xFF),    // Position high byte
        (uint8_t)((relative_pos >> 8) & 0xFF),     // Position low byte
        (uint8_t)(relative_pos & 0xFF)             // Position lowest byte
    };
    
    // CRC 계산
    uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
    
    // 전체 명령어 준비
    uint8_t full_cmd[sizeof(cmd) + 2];
    memcpy(full_cmd, cmd, sizeof(cmd));
    full_cmd[sizeof(cmd)] = crc & 0xFF;            // CRC Low byte
    full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF; // CRC High byte

    // RS485 채널에 따른 전송
    switch(RS485_channel)
    {
        case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
        case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
        case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
    }

    // 디버그 출력 (필요한 경우)
    if(DEBUG_ENABLE == true)
    {
        Serial.print("Move relative position - Motor ID: 0x");
        Serial.print(motor_id, HEX);
        Serial.print(" Relative Position: ");
        Serial.println(relative_pos);
    }
}

void move_to_target_position(uint8_t RS485_channel, uint8_t motor_id, long target_pos) 
{
  // Command frame for absolute position move (Function code 0x7B)
  uint8_t cmd[] = 
  {
    motor_id,                    // Motor ID
    0x7B,                        // Function code for absolute position move
    (uint8_t)((target_pos >> 24) & 0xFF),  // Position highest byte
    (uint8_t)((target_pos >> 16) & 0xFF),  // Position high byte
    (uint8_t)((target_pos >> 8) & 0xFF),   // Position low byte
    (uint8_t)(target_pos & 0xFF)           // Position lowest byte
  };
  
  // CRC calculation
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command with CRC
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;

  // Send via appropriate RS485 channel
  switch(RS485_channel)
  {
    case 1: 
      RS485_Send1(full_cmd, sizeof(full_cmd));
      break;
    case 2: 
      RS485_Send2(full_cmd, sizeof(full_cmd));
      break;         
    case 3: 
      RS485_Send3(full_cmd, sizeof(full_cmd));
      break; 
  }

  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(6500);  // 6.5ms의 수신 대기 시간
  
  // 응답 처리
  int bytes_read = 0;
  bool success = false;
  uint8_t response_buffer[20] = {0};  // 응답을 임시 저장할 버퍼
  long response_position = 0;
  
  // RS485 채널에 따라 데이터 읽기
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 20) 
      {
        response_buffer[bytes_read++] = Serial1.read();
      }
      if(bytes_read > 0)
      {
        success = handle_move_to_position_response(response_buffer, bytes_read, motor_id, &response_position);
      }
      break;
      
    case 2:
      while(Serial2.available() && bytes_read < 20) 
      {
        response_buffer[bytes_read++] = Serial2.read();
      }
      if(bytes_read > 0)
      {
        success = handle_move_to_position_response(response_buffer, bytes_read, motor_id, &response_position);
      }
      break;
      
    case 3:
      while(Serial3.available() && bytes_read < 20) 
      {
        response_buffer[bytes_read++] = Serial3.read();
      }
      if(bytes_read > 0)
      {
        success = handle_move_to_position_response(response_buffer, bytes_read, motor_id, &response_position);
      }
      break;
  }

  // Debug print
  if(DEBUG_ENABLE && !success) 
  {
    Serial.print("Move to target position - Motor ID: 0x");
    Serial.print(motor_id, HEX);
    Serial.print(" Target Position: ");
    Serial.print(target_pos);
    Serial.print(" - Command: ");
    
    for (int i = 0; i < sizeof(full_cmd); i++) 
    {
      printHex(full_cmd[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    if(bytes_read > 0)
    {
      Serial.print("Received response - bytes: ");
      Serial.print(bytes_read);
      Serial.print(" Status: ");
      Serial.println(success ? "Success" : "Failed");
      
      if(success)
      {
        Serial.print("Response position: ");
        Serial.println(response_position);
      }
      
      Serial.print("Response: ");
      for(int i = 0; i < bytes_read; i++)
      {
        printHex(response_buffer[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    else
    {
      Serial.println("No response received");
    }
  }
}


// System voltage 읽기 함수
void readSystemVoltage(uint8_t RS485_channel,uint8_t motor_id)
{
  command_type = 0x11;  // System voltage command type
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x03,       // Function Code (Read Holding Registers)
    0x00, 0x11, // Register Address (System voltage: 0x0011)
    0x00, 0x01  // Number of registers to read (1 register)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  // RS485로 명령 전송
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }
  
  
  // 디버그 출력
  if(DEBUG_ENABLE == false)  return;

  Serial.print("Sent System Voltage Read command to motor ID: 0x");
  Serial.print(motor_id, HEX);
  Serial.print(" - Full command: ");
  
  // 전체 명령 (CRC 포함) 출력
  for (int i = 0; i < sizeof(cmd); i++) 
  {
    printHex(cmd[i]);
    Serial.print(" ");
  }
  printHex(crc & 0xFF);
  Serial.print(" ");
  printHex((crc >> 8) & 0xFF);
  Serial.println();
}


// Read Driver Output Enable status function
bool readDriverOutputEnable(uint8_t RS485_channel, uint8_t motor_id)
{
  command_type = 0x23;  // Unique command type for read driver output enable
  
  // Command frame: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = 
  {
    motor_id,    // Device ID
    0x03,       // Function Code (Read Holding Registers)
    0x00, 0x01, // Register Address (Driver Output Enable: 0x0001)
    0x00, 0x01  // Number of registers to read (1 register)
  };
  
  // CRC calculation
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // Prepare full command with CRC
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;
  
  // Send via appropriate RS485 channel
  switch(RS485_channel) {
    case 1: 
      RS485_Send1(full_cmd, sizeof(full_cmd));
      break;
    case 2: 
      RS485_Send2(full_cmd, sizeof(full_cmd));
      break;         
    case 3: 
      RS485_Send3(full_cmd, sizeof(full_cmd));
      break; 
    default:
      return false;
  }
  
  // Wait for response
  delayMicroseconds(6500);  // 6.5ms wait time for response

  // Process response
  int bytes_read = 0;
  bool driver_output_enable = false;
  bool success = false;
  
  // Read from appropriate channel and process response
  switch(RS485_channel) 
  {
    case 1:
      while(Serial1.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf1[i] = modbus_buf1[i+1];
        }
        uint8_t receivedByte = Serial1.read();
        modbus_buf1[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) 
      {
        success = handleDriverOutputStatusResponse(modbus_buf1, motor_id, &driver_output_enable);
      }
      break;
      
    case 2:
      while(Serial2.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf2[i] = modbus_buf2[i+1];
        }
        uint8_t receivedByte = Serial2.read();
        modbus_buf2[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) 
      {
        success = handleDriverOutputStatusResponse(modbus_buf2, motor_id, &driver_output_enable);
      }
      break;
      
    case 3:
      while(Serial3.available() && bytes_read < 15) 
      {
        for(int i=0; i < NO_DATA-1; i++) 
        {
          modbus_buf3[i] = modbus_buf3[i+1];
        }
        uint8_t receivedByte = Serial3.read();
        modbus_buf3[NO_DATA-1] = receivedByte;
        bytes_read++;
      }
      
      if(bytes_read > 0) 
      {
        success = handleDriverOutputStatusResponse(modbus_buf3, motor_id, &driver_output_enable);
      }
      break;
  }
  
  if(DEBUG_ENABLE && !success) 
  {
    Serial.print("Failed to read Driver Output Enable status for Motor ID ");
    Serial.println(motor_id);
  }
  else if(DEBUG_ENABLE && success)
  {
    Serial.print("Driver Output Enable status for Motor ID ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(driver_output_enable ? "Enabled" : "Disabled");
  }
  
  return driver_output_enable;
}
void readRegister(uint8_t RS485_channel, uint8_t motor_id, uint8_t register_address)
{
  command_type = 14;  // Read command type
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Number of Registers]
  uint8_t cmd[] = 
  {
    motor_id,             // Device ID
    0x03,                // Function Code (Read Holding Registers)
    0x00,                // Register Address High byte
    register_address,    // Register Address Low byte
    0x00,                // Number of registers High byte
    0x01                 // Number of registers to read (1 register)
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송을 위한 전체 프레임 준비
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;         // CRC Low byte
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;  // CRC High byte
  
  // RS485로 명령 전송
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }

  // 디버그 출력
  if(DEBUG_ENABLE == false)  return;

  Serial.print("Reading register 0x");
  Serial.print(register_address, HEX);
  Serial.print(" from motor ID: 0x");
  Serial.println(motor_id, HEX);
  
  Serial.print("Send: ");
  for (int i = 0; i < sizeof(full_cmd); i++) 
  {
    printHex(full_cmd[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  
}

void writeRegister(uint8_t RS485_channel, uint8_t motor_id, uint8_t register_address, uint16_t value)
{
  command_type = 15;  // Write command type
  
  // 명령 프레임 구성: [ID + Function Code + Register Address + Data]
  uint8_t cmd[] = 
  {
    motor_id,             // Device ID
    0x06,                // Function Code (Write Single Register)
    0x00,                // Register Address High byte
    register_address,    // Register Address Low byte
    (uint8_t)(value >> 8),    // Data High byte
    (uint8_t)(value & 0xFF)   // Data Low byte
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송을 위한 전체 프레임 준비
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;         // CRC Low byte
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;  // CRC High byte
  
  // RS485로 명령 전송
  switch(RS485_channel)
  {
    case 1: 
            RS485_Send1(full_cmd, sizeof(full_cmd));
            break;
    case 2: 
            RS485_Send2(full_cmd, sizeof(full_cmd));
            break;         
    case 3: 
            RS485_Send3(full_cmd, sizeof(full_cmd));
            break; 
  }

  // 디버그 출력
  if(DEBUG_ENABLE == false)  return;
  Serial.print("Writing register 0x");
  Serial.print(register_address, HEX);
  Serial.print(" motor ID: 0x");
  Serial.print(motor_id, HEX);
  Serial.print(" value: 0x");
  Serial.println(value, HEX);
  
  Serial.print("Send: ");
  for (int i = 0; i < sizeof(full_cmd); i++) 
  {
    printHex(full_cmd[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  
}

// 편의를 위한 래퍼 함수들
void enableDriverOutput(uint8_t RS485_channel, uint8_t motor_id) 
{
  sendDriverOutputEnable(RS485_channel,motor_id, true);
}

void disableDriverOutput(uint8_t RS485_channel,uint8_t motor_id) 
{
  sendDriverOutputEnable(RS485_channel,motor_id, false);
}

void setMotorSpeed(uint8_t RS485_channel,uint8_t motor_id, int16_t speed)
{
  setMotorTargetSpeed(RS485_channel,motor_id, speed);
}

void Timer2_ISR(void)
{
  //Read Encoder Steering

  
}

// 채널 데이터를 로봇 휠 속도로 변환하는 함수
int16_t convertChannelToWheelSpeed(int ch_data) 
{
  int16_t robot_wheel_speed = 0;
  
  // 중립 상태 (950~1050): 정지
  if (ch_data >= 950 && ch_data <= 1050) 
  {
    robot_wheel_speed = 0;
  }
  // 전진 명령 (200~950)
  else if (ch_data < 950) 
  {
    if (ch_data <= 200) 
    {
      // 최대 전진 (전진 4단계)
      robot_wheel_speed = 600;
    } 
    else if (ch_data <= 400) 
    {
      // 전진 3단계
      robot_wheel_speed = 400;
    }
    else if (ch_data <= 600) 
    {
      // 전진 2단계
      robot_wheel_speed = 300;
    }
    else if (ch_data < 950) 
    {
      // 전진 1단계
      robot_wheel_speed = 200;
    }
  }
  // 후진 명령 (1050~1800)
  else if (ch_data > 1050) 
  {
    if (ch_data >= 1800) 
    {
      // 최대 후진 (후진 4단계)
      robot_wheel_speed = -600;
    }
    else if (ch_data >= 1600) 
    {
      // 후진 3단계
      robot_wheel_speed = -400;
    }
    else if (ch_data >= 1400) 
    {
      // 후진 2단계
      robot_wheel_speed = -300;
    }
    else if (ch_data > 1050) 
    {
      // 후진 1단계
      robot_wheel_speed = -200;
    }
  }
  
  return robot_wheel_speed;
}

// 채널 데이터를 스티어링 각도로 변환하는 함수
int16_t convertChannelToSteeringAngle(int ch_data) 
{
  int16_t steering_angle = 0;
  
  // 중립 상태 (950~1050): 0도
  if (ch_data >= 950 && ch_data <= 1050) 
  {
    steering_angle = 0;
  }
  // 양의 방향 명령 (채널값이 작아짐, 200~950)
  else if (ch_data < 950) 
  {
    if (ch_data <= 200) 
    {
      // 최대 양의 방향 (4단계: +40도)
      steering_angle = 40;
    } 
    else if (ch_data <= 450) 
    {
      // 3단계: +30도
      steering_angle = 30;
    }
    else if (ch_data <= 700) 
    {
      // 2단계: +20도
      steering_angle = 20;
    }
    else if (ch_data < 950) 
    {
      // 1단계: +10도
      steering_angle = 10;
    }
  }
  // 음의 방향 명령 (채널값이 커짐, 1050~1800)
  else if (ch_data > 1050) 
  {
    if (ch_data >= 1800) 
    {
      // 최대 음의 방향 (4단계: -40도)
      steering_angle = -40;
    }
    else if (ch_data >= 1550) 
    {
      // 3단계: -30도
      steering_angle = -30;
    }
    else if (ch_data >= 1300) 
    {
      // 2단계: -20도
      steering_angle = -20;
    }
    else if (ch_data > 1050) 
    {
      // 1단계: -10도
      steering_angle = -10;
    }
  }
  
  return steering_angle;
}


void front_steering_neutral_positioning(void)
{
    float front_steering_angle_adc = read_front_steering_sensor();
}

void rear_steering_neutral_positioning(void)
{
  float rear_steering_angle_adc = read_rear_steering_sensor();
}



// steering 관련 함수

// 스티어링 명령 시작
void start_front_steering_control(float target_angle) 
{
  // 스티어링 상태 업데이트
  front_steering_state.is_moving = true;
  front_steering_state.target_angle = target_angle;
  front_steering_state.target_encoder = 13150 - target_angle*16400;
  front_steering_state.start_time = millis();
  
  // 휠 모터 출력 비활성화 (아직 비활성화되지 않았다면)
  if(!front_steering_state.wheels_disabled && !rear_steering_state.wheels_disabled) 
  {
    if(base_speed <=50) 
    {
      disable_all_wheel_motors();   
    }    
  }
  front_steering_state.wheels_disabled = true;
  
  // 스티어링 명령 전송
  move_to_target_position(2, 1, front_steering_state.target_encoder);
  
  if(DEBUG_ENABLE) 
  {
    Serial.print("Starting front steering movement to angle: ");
    Serial.print(target_angle);
    Serial.print(" (encoder position: ");
    Serial.print(front_steering_state.target_encoder);
    Serial.println(")");
  }
}

void start_rear_steering_control(float target_angle) 
{
  // 스티어링 상태 업데이트
  rear_steering_state.is_moving = true;
  rear_steering_state.target_angle = target_angle;
  rear_steering_state.target_encoder = 15350 + target_angle*16400;
  rear_steering_state.start_time = millis();
  
  // 휠 모터 출력 비활성화 (아직 비활성화되지 않았다면)
  if(!front_steering_state.wheels_disabled && !rear_steering_state.wheels_disabled) 
  {
    if(base_speed <=50) 
    {
      disable_all_wheel_motors();   
    }
  }
  rear_steering_state.wheels_disabled = true;
  
  // 스티어링 명령 전송
  move_to_target_position(2, 2, rear_steering_state.target_encoder);
  
  if(DEBUG_ENABLE) 
  {
    Serial.print("Starting rear steering movement to angle: ");
    Serial.print(target_angle);
    Serial.print(" (encoder position: ");
    Serial.print(rear_steering_state.target_encoder);
    Serial.println(")");
  }
}


// 스티어링 명령 시작 함수 수정 - 좌우 속도 조절
void start_front_steering_control1(float target_angle) 
{
  // 스티어링 상태 업데이트
  front_steering_state.is_moving = true;
  front_steering_state.target_angle = target_angle;
  front_steering_state.target_encoder = 13150 - target_angle*16400;
  front_steering_state.start_time = millis();
  
  // 휠 모터를 비활성화하는 대신 차동 속도를 적용
  //apply_differential_speed(target_angle);
  diff_speed = 50;

  // 스티어링 명령 전송
  move_to_target_position(2, 1, front_steering_state.target_encoder);
  
  if(DEBUG_ENABLE) 
  {
    Serial.print("Starting front steering movement to angle: ");
    Serial.print(target_angle);
    Serial.print(" (encoder position: ");
    Serial.print(front_steering_state.target_encoder);
    Serial.println(")");
  }
}

void start_rear_steering_control1(float target_angle) 
{
  // 스티어링 상태 업데이트
  rear_steering_state.is_moving = true;
  rear_steering_state.target_angle = target_angle;
  rear_steering_state.target_encoder = 15350 + target_angle*16400;
  rear_steering_state.start_time = millis();
  
  // 휠 모터를 비활성화하는 대신 차동 속도를 적용
  apply_differential_speed(target_angle);
  
  // 스티어링 명령 전송
  move_to_target_position(2, 2, rear_steering_state.target_encoder);
  
  if(DEBUG_ENABLE) 
  {
    Serial.print("Starting rear steering movement to angle: ");
    Serial.print(target_angle);
    Serial.print(" (encoder position: ");
    Serial.print(rear_steering_state.target_encoder);
    Serial.println(")");
  }
}

// 스티어링 상태 확인 함수
bool check_steering_status() 
{
  bool front_reached = true;
  bool rear_reached = true;
  unsigned long current_time = millis();
  
  // 전방 스티어링 확인
  if(front_steering_state.is_moving) 
  {
    // 엔코더 값 읽기
    int32_t current_position = sendReadEncoder(2, 1);
    
    // 타겟 위치 도달 확인 (±100 오차 허용)
    long position_diff = abs(current_position - front_steering_state.target_encoder);
    
    if(position_diff <= 100) 
    {
      // 목표 위치 도달
      front_steering_state.is_moving = false;
      if(DEBUG_ENABLE) 
      {
        Serial.print("Front steering reached target angle ");
        Serial.println(front_steering_state.target_angle);
      }
    } 
    else if(current_time - front_steering_state.start_time > front_steering_state.timeout) 
    {
      // 타임아웃 발생
      front_steering_state.is_moving = false;
      if(DEBUG_ENABLE) 
      {
        Serial.print("Front steering timeout - Target: ");
        Serial.print(front_steering_state.target_encoder);
        Serial.print(", Current: ");
        Serial.println(current_position);
      }
    } 
    else 
    {
      // 아직 이동 중
      front_reached = false;
    }
  }
  
  // 후방 스티어링 확인
  if(rear_steering_state.is_moving) 
  {
    // 엔코더 값 읽기
    int32_t current_position = sendReadEncoder(2, 2);
    
    // 타겟 위치 도달 확인 (±100 오차 허용)
    long position_diff = abs(current_position - rear_steering_state.target_encoder);
    
    if(position_diff <= 100) 
    {
      // 목표 위치 도달
      rear_steering_state.is_moving = false;
      if(DEBUG_ENABLE) 
      {
        Serial.print("Rear steering reached target angle ");
        Serial.println(rear_steering_state.target_angle);
      }
    } 
    else if(current_time - rear_steering_state.start_time > rear_steering_state.timeout) 
    {
      // 타임아웃 발생
      rear_steering_state.is_moving = false;
      if(DEBUG_ENABLE) {
        Serial.print("Rear steering timeout - Target: ");
        Serial.print(rear_steering_state.target_encoder);
        Serial.print(", Current: ");
        Serial.println(current_position);
      }
    } 
    else 
    {
      // 아직 이동 중
      rear_reached = false;
    }
  }
  
  // 모든 스티어링이 목표에 도달하고 휠 모터가 비활성화된 상태라면 휠 모터 다시 활성화
  if(front_reached && rear_reached && 
     (front_steering_state.wheels_disabled || rear_steering_state.wheels_disabled)) 
     {
    // 모든 스티어링이 완료되면 휠 모터 다시 활성화  
    enable_all_wheel_motors();
    front_steering_state.wheels_disabled = false;
    rear_steering_state.wheels_disabled = false;
  }
  
  // 모든 스티어링이 목표에 도달했는지 여부 반환
  return front_reached && rear_reached;
}

bool check_steering_status1() 
{
  bool front_reached = true;
  bool rear_reached = true;
  unsigned long current_time = millis();
  
  // 전방 스티어링 확인
  if(front_steering_state.is_moving) 
  {
    // 엔코더 값 읽기
    int32_t current_position = sendReadEncoder(2, 1);
    
    // 타겟 위치 도달 확인 (±100 오차 허용)
    long position_diff = abs(current_position - front_steering_state.target_encoder);
    
    if(position_diff <= 100) 
    {
      // 목표 위치 도달
      front_steering_state.is_moving = false;
      diff_speed = 0;
      if(DEBUG_ENABLE) 
      {
        Serial.print("Front steering reached target angle ");
        Serial.println(front_steering_state.target_angle);
      }
    } 
    else if(current_time - front_steering_state.start_time > front_steering_state.timeout) 
    {
      // 타임아웃 발생
      front_steering_state.is_moving = false;
      diff_speed = 0;
      if(DEBUG_ENABLE) 
      {
        Serial.print("Front steering timeout - Target: ");
        Serial.print(front_steering_state.target_encoder);
        Serial.print(", Current: ");
        Serial.println(current_position);
      }
    } 
    else 
    {
      // 아직 이동 중
      front_reached = false;
    }
  }
  
  // 후방 스티어링 확인
  if(rear_steering_state.is_moving) 
  {
    // 엔코더 값 읽기
    int32_t current_position = sendReadEncoder(2, 2);
    
    // 타겟 위치 도달 확인 (±100 오차 허용)
    long position_diff = abs(current_position - rear_steering_state.target_encoder);
    
    if(position_diff <= 100) 
    {
      // 목표 위치 도달
      rear_steering_state.is_moving = false;
      if(DEBUG_ENABLE) 
      {
        Serial.print("Rear steering reached target angle ");
        Serial.println(rear_steering_state.target_angle);
      }
    } 
    else if(current_time - rear_steering_state.start_time > rear_steering_state.timeout) 
    {
      // 타임아웃 발생
      rear_steering_state.is_moving = false;
      if(DEBUG_ENABLE) {
        Serial.print("Rear steering timeout - Target: ");
        Serial.print(rear_steering_state.target_encoder);
        Serial.print(", Current: ");
        Serial.println(current_position);
      }
    } 
    else 
    {
      // 아직 이동 중
      rear_reached = false;
    }
  }
  
  // 모든 스티어링이 목표에 도달하면 원래 속도로 복원
  if(front_reached && rear_reached) 
  {
    // 스티어링이 움직이는 상태였다가 완료된 경우에만 속도 재설정
    if(front_steering_state.is_moving == false && rear_steering_state.is_moving == false &&
       (diff_speed != 0 || front_steering_state.wheels_disabled || rear_steering_state.wheels_disabled))
    {
      // 차동 속도 초기화
      diff_speed = 0;
      
      // 모터가 정지 상태인지 확인
      bool motors_stopped = true;
      for(int i = 0; i < 4; i++) {
        if(wheel_motor_speed[i] != 0) {
          motors_stopped = false;
          break;
        }
      }
      
      // 정지 상태가 아닐 때만 원래 속도로 복원
      if(!motors_stopped) {
        setMotorTargetSpeed(1, 1, -wheel_motor_speed[0]);
        setMotorTargetSpeed(1, 2, wheel_motor_speed[1]);
        setMotorTargetSpeed(1, 3, -wheel_motor_speed[2]);
        setMotorTargetSpeed(1, 4, wheel_motor_speed[3]);
      } else {
        // 정지 상태이면 모든 모터를 완전히 정지
        setMotorTargetSpeed(1, 1, 0);
        setMotorTargetSpeed(1, 2, 0);
        setMotorTargetSpeed(1, 3, 0);
        setMotorTargetSpeed(1, 4, 0);
      }
      
      if(DEBUG_ENABLE) {
        Serial.println("Steering completed, restored original motor speeds");
      }
    }
    
    front_steering_state.wheels_disabled = false;
    rear_steering_state.wheels_disabled = false;
  }
  
  // 모든 스티어링이 목표에 도달했는지 여부 반환
  return front_reached && rear_reached;
}


// 스티어링 움직임이 완료됐는지 확인하는 함수
bool is_steering_movement_complete() 
{
  return !front_steering_state.is_moving && !rear_steering_state.is_moving;
}

// 모든 휠 모터 출력 비활성화 함수
void disable_all_wheel_motors() 
{
  for(int motor_id = 1; motor_id <= 4; motor_id++) 
  {
    sendDriverOutputEnable(1, motor_id, false);
    delay(50); // 약간의 딜레이
  }
  if(DEBUG_ENABLE) {
    Serial.println("All wheel motors disabled");
  }
}

// 모든 휠 모터 출력 활성화 함수
void enable_all_wheel_motors() 
{
  for(int motor_id = 1; motor_id <= 4; motor_id++) 
  {
    sendDriverOutputEnable(1, motor_id, true);
    delay(10); // 약간의 딜레이
  }
  if(DEBUG_ENABLE) {
    Serial.println("All wheel motors enabled");
  }
}

// 차동 속도 적용 함수 추가
void apply_differential_speed(float angle)
{
  // 각도의 절대값 계산
  float abs_angle = abs(angle);
  int max_speed = 500;
  
  // 기존 wheel_motor_speed 값의 평균을 계산
  int16_t avg_speed = (abs(wheel_motor_speed[0]) + abs(wheel_motor_speed[1]) + 
                       abs(wheel_motor_speed[2]) + abs(wheel_motor_speed[3])) / 4;
  
  // 차동 속도 계산 (각도가 커질수록 차동 속도도 증가)
  diff_speed = (int16_t)(abs_angle * 10); // 각도 1도당 10씩 속도 차이
  
  // 스티어링 움직임이 시작됨을 표시
  front_steering_state.is_moving = true;
  rear_steering_state.is_moving = true;
  
  // 평균 속도가 0에 가까울 때 (제자리 회전 모드)
  if(avg_speed < 50) 
  {
    // 제자리에서 회전하기 위한 코드
    if(angle > 0) 
    {
      // 오른쪽으로 조향하는 경우 (양수 각도)
      // 왼쪽 바퀴는 전진, 오른쪽 바퀴는 후진
      int16_t rotation_speed = diff_speed;
      
      // 제자리 회전 속도 제한
      rotation_speed = constrain(rotation_speed, 0, max_speed/2);
      
      // 모터 속도 설정
      setMotorTargetSpeed(1, 1, -rotation_speed);  // 왼쪽 앞 (전진)
      setMotorTargetSpeed(1, 2, -rotation_speed);  // 오른쪽 앞 (후진)
      setMotorTargetSpeed(1, 3, -rotation_speed);  // 왼쪽 뒤 (전진)
      setMotorTargetSpeed(1, 4, -rotation_speed);  // 오른쪽 뒤 (후진)
    }
    else if(angle < 0) 
    {
      // 왼쪽으로 조향하는 경우 (음수 각도)
      // 오른쪽 바퀴는 전진, 왼쪽 바퀴는 후진
      int16_t rotation_speed = diff_speed;
      
      // 제자리 회전 속도 제한
      rotation_speed = constrain(rotation_speed, 0, max_speed/2);
      
      // 모터 속도 설정
      setMotorTargetSpeed(1, 1, rotation_speed);   // 왼쪽 앞 (후진)
      setMotorTargetSpeed(1, 2, rotation_speed);   // 오른쪽 앞 (전진)
      setMotorTargetSpeed(1, 3, rotation_speed);   // 왼쪽 뒤 (후진)
      setMotorTargetSpeed(1, 4, rotation_speed);   // 오른쪽 뒤 (전진)
    }
    else 
    {
      // 각도가 0인 경우 모든 모터 정지
      setMotorTargetSpeed(1, 1, 0);
      setMotorTargetSpeed(1, 2, 0);
      setMotorTargetSpeed(1, 3, 0);
      setMotorTargetSpeed(1, 4, 0);
    }
  }
  // 움직이면서 조향하는 경우
  else 
  {
    if(angle > 0) 
    {
      // 오른쪽으로 조향하는 경우 (양수 각도)
      // 왼쪽 바퀴(1, 3번)는 더 빠르게, 오른쪽 바퀴(2, 4번)는 더 느리게
      int16_t left_speed = wheel_motor_speed[0] + diff_speed;
      int16_t right_speed = wheel_motor_speed[1] - diff_speed;
      
      // 속도 범위 제한
      left_speed = constrain(left_speed, -max_speed, max_speed);
      right_speed = constrain(right_speed, -max_speed, max_speed);
      
      // 모터 속도 설정
      setMotorTargetSpeed(1, 1, -left_speed);  // 왼쪽 앞
      setMotorTargetSpeed(1, 2, right_speed);  // 오른쪽 앞
      setMotorTargetSpeed(1, 3, -left_speed);  // 왼쪽 뒤
      setMotorTargetSpeed(1, 4, right_speed);  // 오른쪽 뒤
    }
    else if(angle < 0) 
    {
      // 왼쪽으로 조향하는 경우 (음수 각도)
      // 오른쪽 바퀴(2, 4번)는 더 빠르게, 왼쪽 바퀴(1, 3번)는 더 느리게
      int16_t left_speed = wheel_motor_speed[0] - diff_speed;
      int16_t right_speed = wheel_motor_speed[1] + diff_speed;
      
      // 속도 범위 제한
      left_speed = constrain(left_speed, -max_speed, max_speed);
      right_speed = constrain(right_speed, -max_speed, max_speed);
    
      // 모터 속도 설정
      setMotorTargetSpeed(1, 1, -left_speed);  // 왼쪽 앞
      setMotorTargetSpeed(1, 2, right_speed);  // 오른쪽 앞
      setMotorTargetSpeed(1, 3, -left_speed);  // 왼쪽 뒤
      setMotorTargetSpeed(1, 4, right_speed);  // 오른쪽 뒤
    }
    else 
    {
      // 각도가 0인 경우 차동 속도 없이 원래 속도로 설정
      diff_speed = 0;
      setMotorTargetSpeed(1, 1, -wheel_motor_speed[0]);  // 왼쪽 앞
      setMotorTargetSpeed(1, 2, wheel_motor_speed[1]);   // 오른쪽 앞
      setMotorTargetSpeed(1, 3, -wheel_motor_speed[2]);  // 왼쪽 뒤
      setMotorTargetSpeed(1, 4, wheel_motor_speed[3]);   // 오른쪽 뒤
    }
  }
  
  if(DEBUG_ENABLE && diff_speed != 0) {
    Serial.print("Applied differential speed: ");
    Serial.println(diff_speed);
  }
}
void setup()
{

  // SPI
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  pinMode(TX_485_1,OUTPUT);  //485TX1
  pinMode(TX_485_2,OUTPUT);  //485TX2 
  pinMode(TX_485_3,OUTPUT);  //485TX3

  pinMode(D22, INPUT);
  pinMode(D23, INPUT);
  pinMode(D24, INPUT);
  pinMode(D25, INPUT);
  pinMode(D26, INPUT);
  pinMode(D27, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INT_1), INT1_ISR, RISING);
  
  pinMode(D42, INPUT);
  pinMode(D43, INPUT);
  pinMode(D44, INPUT);
  pinMode(D45, INPUT);
  pinMode(D46, INPUT);
  pinMode(D47, INPUT); 
  //attachInterrupt(digitalPinToInterrupt(INT_2), INT2_ISR, RISING);
   
  Serial.begin(115200);
  Serial.println("start");
  digitalWrite(TX_485_1, LOW);  // 초기 상태는 수신 모드
  digitalWrite(TX_485_2, LOW);  // 초기 상태는 수신 모드
  digitalWrite(TX_485_3, LOW);  // 초기 상태는 수신 모드
  
  Serial1.begin(19200); // Serial Port 1
  Serial2.begin(19200); // Serial Port 2
  Serial3.begin(19200); // Serial Port 3  
  
  delay(1000);

  // CAN
  /* 
  if(CAN0.begin(CAN_500KBPS) == CAN_OK) 
  {
    Serial.print("Can init ok!!\r\n"); // 500Kbs의 속도로 CAN버스를 초기화 합니다.
  }
  else
  {
    Serial.print("Can init fail!!\r\n");
  } 
  */
  
  // sendModbusEnable(1,1);
  // sendModbusEnable(2,1);
  // sendModbusEnable(3,1);
  
  sendModbusEnable(1,1,true);
  delay(70);
  sendModbusEnable(1,2,true);
  delay(70);
  sendModbusEnable(1,3,true);
  delay(70);
  sendModbusEnable(1,4,true);
  delay(70);

  enable_all_wheel_motors();
  delay(70);
  
  setMotorTargetSpeed(1,1,0);  //기능 테스트 완료
  delay(100);
  setMotorTargetSpeed(1,2,0);  //기능 테스트 완료
  delay(100);
  setMotorTargetSpeed(1,3,0);  //기능 테스트 완료
  delay(100);   
  setMotorTargetSpeed(1,4,0);  //기능 테스트 완료
  delay(0100);
  
  disable_all_wheel_motors();
  delay(1000);

  Serial.println(readDriverOutputEnable(1,1));

  sendModbusEnable(2,1,true);
  delay(10);
  
  sendModbusEnable(2,2,true);
  delay(10);

  sendDriverOutputEnable(2,1,true);
  delay(10);

  sendDriverOutputEnable(2,2,true);

  delay(10);

  setMotorTargetSpeed(2,1, 1000);
  delay(10);

  setMotorTargetSpeed(2,2, 1000);
  delay(10);

  front_steering_control(0);
  delay(10);
  rear_steering_control(0);
  
  delay(1000);
  
  allRelaysOn(2);  // RS485 2 channel 에 연결

  //sendDriverOutputEnable(2,1,true); 
  //delay(100);
  //MsTimer2 
  //MsTimer2::set(500, Timer2_ISR); // 200ms period
  //MsTimer2::start();

  check_front_steering();  
 
  delay(500);

  check_rear_steering();

  delay(500);

  enable_all_wheel_motors();
  delay(70);

  sendDriverOutputEnable(2,1,true);
  delay(10);

  sendDriverOutputEnable(2,2,true);

  delay(10);

}

void loop1()
{
  unsigned long int currentTime = millis();	// by 시간 함수
  interval = 200;
  uint16_t delay_time = 3800;  // 60

  if (currentTime - preTime > interval) 
  {
    preTime = currentTime;

    Serial.print("Front : ");
    Serial.print(read_front_steering_sensor_adc());
    Serial.print(" ");
    Serial.println(analogRead(A1)); // Read analog value
   
    Serial.print("Rear : ");
    Serial.print(read_rear_steering_sensor_adc());
    Serial.print(" ");
    Serial.println(analogRead(A2)); // Read analog value
   
    
  }
}

void loop()
{

  unsigned long int currentTime = millis();	// by 시간 함수
  uint16_t delay_time = 3000;  // 60
  int ch1_data;
  int ch2_data;
  int robot_wheel_speed = 0;
  int robot_steering_angle = 0;
 
  if (currentTime - preTime > interval) 
  {
    preTime = currentTime;
      
    //SPI data send
    sendData_SPI();
    //run_mode = FULL_AUTO;

    if(run_mode  == MANUAL)
    {
        ch1_data =  (receiveProtocol.ch1_H << 8) | receiveProtocol.ch1_L;

        // CH1 값을 기반으로 휠 속도 계산
        ch1_data = (receiveProtocol.ch1_H << 8) | receiveProtocol.ch1_L;
        
        base_speed = robot_wheel_speed = convertChannelToWheelSpeed(ch1_data);

        // update 부분 추가 해야 함 
        int16_t wheel_motor_speed_new[4] = { 0, 0, 0, 0};
        diff_speed = 0;
        wheel_motor_speed_new[0] =   robot_wheel_speed - diff_speed;     
        wheel_motor_speed_new[1] =   robot_wheel_speed + diff_speed; 
        wheel_motor_speed_new[2] =   robot_wheel_speed - diff_speed ; 
        wheel_motor_speed_new[3] =   robot_wheel_speed + diff_speed;
        
        /*
        for(int i=0; i<4; i++)
        {
          Serial.print(wheel_motor_speed_new[i]);
          Serial.print(" ");

        }
        Serial.println("");
        */

        isWheelSpeedCommandUpdated(wheel_motor_speed_new);

        for(int i = 0; i<4 ;i++)
        {
          if(wheel_speed_command_update[i] == true)
          {
            if(i == 0) wheel_motor_speed[0]  =  -wheel_motor_speed[0];
            if(i == 2) wheel_motor_speed[2]  =  -wheel_motor_speed[2];
            
            setMotorTargetSpeed(1,i+1,wheel_motor_speed[i]);  //기능 테스트 완료
            wheel_speed_command_update[i] = false;
          }
        }

        ch2_data =  (receiveProtocol.ch2_H << 8) | receiveProtocol.ch2_L;  

        robot_steering_angle = convertChannelToSteeringAngle(ch2_data);


        int16_t steering_angle_new[2] = { 0, 0};

        steering_angle_new[0] = robot_steering_angle;
        steering_angle_new[1] = robot_steering_angle;

        isSteeringAngleommandUpdated(steering_angle_new);

        if(steering_mode == CRAB_MODE)
        {
          for(int i = 0; i<2 ;i++)
          {
            if(steering_angle_command_update[i] == true) 
            {
              switch(i)
              {
                 case 0: 
                          start_front_steering_control(steering_angle[0]);
                          steering_angle_command_update[0] = false;         
                         break;
                 case 1 :
                          start_rear_steering_control(steering_angle[1]);
                          steering_angle_command_update[1] = false;      
                          break;        
              }  
            }
          }  
        }

        else if(steering_mode == TANK_MODE) 
        {

          for(int i = 0; i<2 ;i++)
          {
            if(steering_angle_command_update[i] == true) 
            {
              switch(i)
              {
                 case 0: 
                          //front_steering_control(steering_angle[0]);
                          start_front_steering_control(steering_angle[0]);

                          steering_angle_command_update[0] = false;        
                         break;
                 case 1 :
                          //rear_steering_control(-steering_angle[1]);
                          start_rear_steering_control(-steering_angle[1]);
                          
                          steering_angle_command_update[1] = false;       
                          break;        
              }  
            }
          }

        }

        else
        {
          if(steering_angle_command_update[0] == true) 
          {
            //front_steering_control(steering_angle[0]);
            start_front_steering_control(steering_angle[0]);

            
            steering_angle_command_update[0] = false;  
          }
        }

        //Serial.print("Steering angle :  ");
        //Serial.println(robot_steering_angle);        
    }
    else if(run_mode  == SEMI_AUTO)
    {
       // 스티어링 명령어 
      for(int i = 0; i<2 ;i++)
      {

        ch1_data =  (receiveProtocol.ch1_H << 8) | receiveProtocol.ch1_L;

        // CH1 값을 기반으로 휠 속도 계산
        ch1_data = (receiveProtocol.ch1_H << 8) | receiveProtocol.ch1_L;
        
        base_speed = robot_wheel_speed = convertChannelToWheelSpeed(ch1_data);

        // update 부분 추가 해야 함 
        int16_t wheel_motor_speed_new[4] = { 0, 0, 0, 0};
        diff_speed = 0;
        wheel_motor_speed_new[0] =   robot_wheel_speed - diff_speed;     
        wheel_motor_speed_new[1] =   robot_wheel_speed + diff_speed; 
        wheel_motor_speed_new[2] =   robot_wheel_speed - diff_speed ; 
        wheel_motor_speed_new[3] =   robot_wheel_speed + diff_speed;
        
        isWheelSpeedCommandUpdated(wheel_motor_speed_new);

        for(int i = 0; i<4 ;i++)
        {
          if(wheel_speed_command_update[i] == true)
          {
            if(i == 0) wheel_motor_speed[0]  =  -wheel_motor_speed[0];
            if(i == 2) wheel_motor_speed[2]  =  -wheel_motor_speed[2];
            
            setMotorTargetSpeed(1,i+1,wheel_motor_speed[i]);  //기능 테스트 완료
            wheel_speed_command_update[i] = false;
          }
        }

        if(steering_angle_command_update[i] == true) 
        {
          switch(i)
          {
             case 0: 
                      //front_steering_control(steering_angle[0]);
                      start_front_steering_control(steering_angle[0]);
                      
                      steering_angle_command_update[0] = false;                                      
                     break;
             case 1 :
                      //rear_steering_control(steering_angle[1]);
                      start_rear_steering_control(steering_angle[1]);
                      steering_angle_command_update[1] = false;                      
                      break;        
          }  
        }
      }

      

      
    }
    else //(run_mode  == FULL_AUTO)
    {

      base_speed = (int16_t)(wheel_motor_speed[0] + wheel_motor_speed[1] + wheel_motor_speed[2] + wheel_motor_speed[3]) / 4;
      //속도 명령어      
      for(int i = 0; i<4 ;i++)
      {
        if(wheel_speed_command_update[i] == true)
        {
          if(i == 0) wheel_motor_speed[0]  =  -wheel_motor_speed[0];
          if(i == 2) wheel_motor_speed[2]  =  -wheel_motor_speed[2];
          
          setMotorTargetSpeed(1,i+1,wheel_motor_speed[i]);  //기능 테스트 완료
          //delayMicroseconds(delay_time);        
          wheel_speed_command_update[i] = false;
        }
      }
    
      // 스티어링 명령어 
      for(int i = 0; i<2 ;i++)
      {
        if(steering_angle_command_update[i] == true) 
        {
          switch(i)
          {
             case 0: 
                      start_front_steering_control(steering_angle[0]);
                      steering_angle_command_update[0] = false; 
                     break;
             case 1 :
                      start_rear_steering_control(steering_angle[1]);
                      steering_angle_command_update[1] = false;
                      break;        
          }  
        }
     }
    
    }
    
    //sendReadEncoder(2,1);
    //delay(delay_time); 
    //sendReadEncoder(2,2);  
    //delay(delay_time);
    
    // 스티어링 상태 확인
    check_steering_status();

    sendReadEncoder(1,1);
    sendReadEncoder(1,2);
    sendReadEncoder(1,3);
    sendReadEncoder(1,4);
    
    sendReadEncoder(2,1);
    sendReadEncoder(2,2);
    
    // steering motor driver check
    
    for(int i =1 ; i<= 2; i++)
    {
      if ( readDriverOutputEnable(2, i) == 0 )
      {
        sendDriverOutputEnable(2,i,true);
      }

    }
    
    /*
    Serial.print("Front : ");
    Serial.print(read_front_steering_sensor_adc());
    Serial.print(" ");
    Serial.println(analogRead(A1)); // Read analog value
   
    Serial.print("Rear : ");
    Serial.print(read_rear_steering_sensor_adc());
    Serial.print(" ");
    Serial.println(analogRead(A2)); // Read analog value
   
  
    //Serial.print( readDriverOutputEnable(2, 1) );
    */

     
  }
  
}

void INT1_ISR(void)
{
   if(read_limit_switch_front_steering_left() == true)
   {

   }
   if(read_limit_switch_front_steering_right() == true)
   {

   }  
}

void INT2_ISR(void)
{
  
}

bool read_modbus_enable_handler_485(uint8_t* data)
{
  
  // Check for valid Modbus enable response pattern
  for(int i = NO_DATA-7; i >= 0; i--)  
  {  
    if( (data[i+1] == 0x06) && (data[i+2] == 0x00) && (data[i+3] == 0x00) ) 
    {
      
      uint16_t receivedCRC = (data[i+7] << 8) | data[i+6];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 6);
      
      if(receivedCRC == calculatedCRC) 
      {        
        bool isEnabled = (data[i+5] == 0x01);
        //Serial.println(isEnabled ? "Modbus Enable" : "Modbus Disable");

        // Clear the buffer after successful protocol reception
        memset(data, 0, NO_DATA);
        
        return isEnabled;
      }      
    }
  }
  
  // Return false if no valid response was found
  return false;
}

bool read_drive_output_enable_handler_485(uint8_t* data)
{
  
  //Serial.println("read_drive_output_enable_handler_485");
  for(int i = NO_DATA-7; i >= 0 ;i--)  
  {  
    if( (data[i+1] == 0x06) && (data[i+2] == 0x00) && (data[i+3] == 0x01) ) 
    {
      
      uint16_t receivedCRC = (data[i+7] << 8) | data[i+6];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 6);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
      //Serial.println("");
      if(receivedCRC == calculatedCRC) 
      {        
        bool isEnabled = (data[i+5] & 0x01);
        //Serial.println(isEnabled ? "Driver Output Enable" : "Driver Output Disable"); 
        // Clear the buffer after successful protocol reception
        memset(data, 0, NO_DATA);      
        return isEnabled;        
      }      
    }
  }
  // Return false if no valid response was found
  return false;
}


bool read_encoder_handler_485_1(uint8_t* data, int32_t *encoder_value)
{
  /*
  for(int i = 0; i < NO_DATA; i++)
  {
    printHex(data[i]);
    Serial.print(" ");
  }
  Serial.println();
  */

    // Check for valid encoder read response pattern
  for(int i = NO_DATA-9; i >= 0; i--)
  {
      // Check if this is an encoder read response:
      // Function code (0x03) and byte count (0x04)

      if((data[i+1] == 0x03) && (data[i+2] == 0x04))
      {
        // Calculate CRC for the received data
        uint16_t receivedCRC = (data[i+8] << 8) | data[i+7];
        uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 7);
        
        //Serial.println(receivedCRC);
        //Serial.println(calculatedCRC);
        if(receivedCRC == calculatedCRC)
        {
            // Extract encoder value (4 bytes, Little Endian order)
            int32_t encoder_val = (((uint32_t)data[i+4] << 0)  |   // LSB
                                  ((uint32_t)data[i+3] << 8)    |
                                  ((uint32_t)data[i+6] << 16)   |
                                  ((uint32_t)data[i+5] << 24));     // MSB
            
            // Convert to signed 32-bit integer if necessary
            if(encoder_val & 0x80000000)  // If highest bit is 1 (negative number)
            {
                encoder_val = -(~encoder_val & 0xFFFFFFFF) - 1;
            }
            
            // Store the result in the output parameter
            *encoder_value = encoder_val;
            
            if( (data[i+0]>=1) &&(data[i+0]<=4) )
            {
              encoder_pos_wheel[data[i+0]-1] = encoder_val;                 
            }
            // Clear the buffer after successful protocol reception
            memset(data, 0, NO_DATA);
            
            return true;
        }
      }
  }
    
    // Return false if no valid response was found
    return false;
}

bool read_encoder_handler_485_2(uint8_t* data, int32_t *encoder_value)
{
  // Check for valid encoder read response pattern
  for(int i = NO_DATA-9; i >= 0; i--)
  {
      // Check if this is an encoder read response:
      // Function code (0x03) and byte count (0x04)
      if((data[i+1] == 0x03) && (data[i+2] == 0x04))
      {
          // Calculate CRC for the received data
          uint16_t receivedCRC = (data[i+8] << 8) | data[i+7];
          uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 7);
          
          if(receivedCRC == calculatedCRC)
          {
              // Extract encoder value (4 bytes, Little Endian order)
              int32_t encoder_val = (((uint32_t)data[i+4] << 0)  |   // LSB
                                   ((uint32_t)data[i+3] << 8)    |
                                   ((uint32_t)data[i+6] << 16)   |
                                   ((uint32_t)data[i+5] << 24));     // MSB
              
              // Convert to signed 32-bit integer if necessary
              if(encoder_val & 0x80000000)  // If highest bit is 1 (negative number)
              {
                  encoder_val = -(~encoder_val & 0xFFFFFFFF) - 1;
              }
              
              // Store the result in the output parameter
              *encoder_value = encoder_val;
              if( (data[i+0]>=1) &&(data[i+0]<=2) )
              {
                encoder_pos_steering[data[i+0]-1] = encoder_val; 
              }
              // Clear the buffer after successful protocol reception
              memset(data, 0, NO_DATA);
              
              return true;
          }
      }
  }
  
  // Return false if no valid response was found
  return false;
}

bool read_encoder_handler_485_3(uint8_t* data, int32_t *encoder_value)
{


}

bool read_voltage_handler_485(uint8_t* data, float *voltage)
{
  
  //0103023FFE2834
  //Serial.println("voltage: "); 
  //Serial.println(NO_DATA-7); 
  for(int i = NO_DATA-7; i >= 0 ;i--)  
  {
  //  Serial.print("start : ");        Serial.print(i); Serial.print(" ");
  //  Serial.print(data[i+0],HEX);     Serial.print(" ");
  //  Serial.print(data[i+1],HEX);     Serial.print(" ");
  //  Serial.println(data[i+2],HEX);
    if( (data[i+0] == 0x01) && (data[i+1] == 0x03) && (data[i+2] == 0x02)) 
    {
     
      uint16_t receivedCRC = (data[i+6] << 8) | data[i+5];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 5);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
     
      if(receivedCRC == calculatedCRC) 
      {        
        uint16_t raw_value = (data[i+3] << 8) | data[i+4];
        *voltage = raw_value / 327.0f;
        // Clear the buffer after successful protocol reception
        memset(data, 0, NO_DATA);
        return true;
      }
      
    }
  }
  return false;
}



bool decode_uart0_protocal(uint8_t* data)
{
   //명령어  11 byte  
  for(int i = UART_BUFFER_SIZE-10; i >= 0 ;i--)  
  {
    /*
    Serial.print(i);
    Serial.print(data[i+0],HEX);
    Serial.println("");
    Serial.print("start : ");        Serial.print(i); Serial.print(" ");
    Serial.print(data[i+0],HEX);     Serial.print(" ");
    Serial.print(data[i+1],HEX);     Serial.print(" ");
    Serial.println(data[i+2],HEX);
    */
     
     //if(i== (UART_BUFFER_SIZE-10))   Serial.write(data[i+0]);

     /*
        digitalWrite(TX_485_2, HIGH);    // 송신 모드로 전환
        delayMicroseconds(10);          // 모드 전환을 위한 짧은 대기 시간
        Serial2.print("OK"); 
        */


    // ROBOT Run
    if( (data[i+0] == '#') && (data[i+1] == 'R') && (data[i+2] == 'S') && (data[i+10] == '*')) 
    {
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);     

      if(receivedCRC == calculatedCRC) 
      { 
        if(data[i+3] == 0x00)
        {
          int16_t motor_speed = (data[i+4] << 8) | data[i+5];
          int16_t wheel_motor_speed_new[4] = { 0, 0, 0, 0};

          wheel_motor_speed_new[0] = motor_speed;
          wheel_motor_speed_new[1] = motor_speed;
          wheel_motor_speed_new[2] = motor_speed;
          wheel_motor_speed_new[3] = motor_speed;
          
          isWheelSpeedCommandUpdated(wheel_motor_speed_new);
          memset(data,0,UART_BUFFER_SIZE);   
          //motor_all_speed(motor_speed);        
        }
      }
    }
    //motor 구동 명령어 
    if( (data[i+0] == '#') && (data[i+1] == 'S') && (data[i+2] == 'P') && (data[i+10] == '*')) 
    {
                
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
      
      
      
      if(receivedCRC == calculatedCRC) 
      {        
        int16_t motor_speed = (data[i+3] << 8) | data[i+4];
        
        //wheel_motor_speed[data[i+3]-1] = motor_speed;
        //Serial.print(wheel_motor_speed[data[i+3]-1]); 
        
        return true;
      }      
    }
    
    //motor 구동 명령어 stop     
    if( (data[i+0] == '#') && (data[i+1] == 'S') && (data[i+2] == 'S') && (data[i+10] == '*')) 
    {
                 
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
      
      if(receivedCRC == calculatedCRC) 
      {        
        int16_t motor_speed = (data[i+3] << 8) | data[i+4];
        
        //wheel_motor_speed[data[i+3]-1] = motor_speed;
        //Serial.print(wheel_motor_speed[data[i+3]-1]); 

        //motor_all_stop();
        return true;
      }      
    }
     
    //robot steering 구동 명령어
    if( (data[i+0] == '#') && (data[i+1] == 'R') && (data[i+2] == 'T') && (data[i+10] == '*')) 
    {
      //Serial.write(0x01);
      //Serial.write(0x02);
            
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
     
      if(receivedCRC == calculatedCRC) 
      { 
        float target_angle =  (data[i+4] << 8) | data[i+3];

        if(data[i+3] == 0x00)
        {
          int16_t steering_angle1 = (data[i+4] << 8) | data[i+5];
          int16_t steering_angle_new[2] = { 0, 0};
          if(data[i+7] == 0x01)
          {
            steering_angle_new[0] = steering_angle1;
            steering_angle_new[1] = steering_angle1;
          }
          else if(data[i+7] == 0x02)
          {
            steering_angle_new[0] = steering_angle1;
            steering_angle_new[1] = -steering_angle1;
          } 
          else if(data[i+7] == 0x03)
          {
            steering_angle_new[0] = steering_angle1;
            steering_angle_new[1] = 0;
          }
          else
          { 
            steering_angle_new[0] = steering_angle1;
            steering_angle_new[1] = 0;
          }
          
          // test 
          
          isSteeringAngleommandUpdated(steering_angle_new);          
          memset(data,0,UART_BUFFER_SIZE);                   
        }



        return true;
      }//if(receivedCRC == calculatedCRC)   
    }

    //Limit switch status 
    if( (data[i+0] == '#') && (data[i+1] == 'R') && (data[i+2] == 'L') && (data[i+10] == '*')) 
    {
                  
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
      
      if(receivedCRC == calculatedCRC) 
      { 
        send_Limit_Switch_Status();
        return true;
      }      
    }

    //Robot Encoder Read
    if( (data[i+0] == '#') && (data[i+1] == 'R') && (data[i+2] == 'E') && (data[i+10] == '*')) 
    {
      
      //Serial.write(0x05);
      //Serial.write(0x06);        
              
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);
      //Serial.println(receivedCRC, HEX);
      //Serial.println(calculatedCRC, HEX);
      
      if(receivedCRC == calculatedCRC) 
      {         
        send_wheel_encoder_status(1); 
        send_wheel_encoder_status(2); 
        send_wheel_encoder_status(3); 
        send_wheel_encoder_status(4); 
        memset(data, 0, NO_DATA);
        return true;
      }      
    }

    //robot steering angle read
    if( (data[i+0] == '#') && (data[i+1] == 'R') && (data[i+2] == 'A') && (data[i+10] == '*')) 
    {
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);      
      /*
      digitalWrite(TX_485_2, HIGH);    // 송신 모드로 전환
      delayMicroseconds(100);          // 모드 전환을 위한 짧은 대기 시간
      Serial2.print("ok3");
      delayMicroseconds(1000);          // 모드 전환을 위한 짧은 대기 시간
      digitalWrite(TX_485_2, LOW);    // 송신 모드로 전환
      */
      if(receivedCRC == calculatedCRC) 
      {         
        //RS485_Send2(&data[i+0],1);

        send_steering_encoder_status(1);
        send_steering_encoder_status(2);        
        memset(data, 0, NO_DATA);
        return true;
      }
    }    


    // robot run mode 
    if( (data[i+0] == '#') && (data[i+1] == 'R') && (data[i+2] == 'M') && (data[i+10] == '*')) 
    {
      uint16_t receivedCRC = (data[i+9] << 8) | data[i+8];
      uint16_t calculatedCRC = CRC16_MODBUS(&data[i], 8);      
      /*
        digitalWrite(TX_485_2, HIGH);    // 송신 모드로 전환
        delayMicroseconds(100);          // 모드 전환을 위한 짧은 대기 시간
        Serial2.print("ok3");
        delayMicroseconds(1000);          // 모드 전환을 위한 짧은 대기 시간
        digitalWrite(TX_485_2, LOW);    // 송신 모드로 전환
      */
      if(receivedCRC == calculatedCRC) 
      {        
        robot_steering_mode =  data[i+3];
        /*   
        digitalWrite(TX_485_2, HIGH);    // 송신 모드로 전환
        delayMicroseconds(100);          // 모드 전환을 위한 짧은 대기 시간
        Serial2.print("crab");
        delay(100);          // 모드 전환을 위한 짧은 대기 시간
        digitalWrite(TX_485_2, LOW);    // 송신 모드로 전환
        */
        return true;
      
      }
    }

  }
  return false;
}

  
void serialEvent() 
{
  while (Serial.available()) 
  {
    for(int i=0; i < UART_BUFFER_SIZE-1; i++) 
    {
        readBuf[i] =   readBuf[i+1];
    }
    
    uint8_t receivedByte = Serial.read();
    readBuf[UART_BUFFER_SIZE-1] = receivedByte;   
    //Serial.print(receivedByte, HEX);    
    //Serial.write(receivedByte);  // 바이트 그대로 에코
    decode_uart0_protocal(readBuf);

  }  
}


void isWheelSpeedCommandUpdated(int16_t *wheel_motor_speed_new)
{
  int i =0;
 
  for(i=0;i<4;i++)
  {
    if(wheel_motor_speed_new[i]!= wheel_motor_speed[i])
    {
      wheel_speed_command_update[i] = true;      
    }
    else
    {
      wheel_speed_command_update[i] = false;
    }
    wheel_motor_speed[i] = wheel_motor_speed_new[i];
  }
  /*
  digitalWrite(TX_485_2, HIGH);    // 송신 모드로 전환
  delayMicroseconds(10);          // 모드 전환을 위한 짧은 대기 시간
  Serial2.print(wheel_speed_command_update[0]); 
  Serial2.print(wheel_speed_command_update[1]); 
  Serial2.print(wheel_speed_command_update[2]); 
  Serial2.print(wheel_speed_command_update[3]); 
  */
}

void isSteeringAngleommandUpdated(int16_t *steering_angle_new)
{  
  int i =0;
      
  for(i=0;i<2;i++)
  {
          
    if(steering_angle_new[i]!= steering_angle[i])
    {
      steering_angle_command_update[i] = true;
    }
    else
    {
      steering_angle_command_update[i] = false;
    }
    steering_angle[i] = steering_angle_new[i];
  }
}

void motor_front_left_speed(int16_t speed)
{
  setMotorTargetSpeed(1,1, -speed);
}

void motor_front_right_speed(int16_t speed)
{
  setMotorTargetSpeed(1,2, speed);
}

void motor_rear_left_speed(int16_t speed)
{
  setMotorTargetSpeed(1,3, -speed);
}


void motor_rear_right_speed(int16_t speed)
{
  setMotorTargetSpeed(1,4, speed);
}

void motor_all_speed(int16_t speed)
{

  motor_front_left_speed(speed);
  delay(70);
  
  motor_front_right_speed(speed);
  delay(70);

  motor_rear_left_speed(speed);
  delay(70);
  
  motor_rear_right_speed(speed);
  delay(70);  
}

void motor_all_stop(void)
{
  uint16_t delay_time = 60;
  setMotorTargetSpeed(1,1,0);
  delay(delay_time);
  setMotorTargetSpeed(1,2,0);
  delay(delay_time);
  setMotorTargetSpeed(1,3,0);
  delay(delay_time);
  setMotorTargetSpeed(1,4,0);
  delay(delay_time);
}


void motor_all_output_disable(void)
{
  uint16_t delay_time = 60;
  sendDriverOutputEnable(1,1, false);
  delay(delay_time);
  sendDriverOutputEnable(1,2, false);
  delay(delay_time);
  sendDriverOutputEnable(1,3, false);
  delay(delay_time);
  sendDriverOutputEnable(1,4, false);
  delay(delay_time);
}


void front_steering_control(float target_angle)
{
  long front_target_encoder = 13150 - target_angle*16400;
  //front_target_encoder  = 13150;
  move_to_target_position(2, 1, front_target_encoder ) ;
}

void rear_steering_control(float target_angle)
{
  long rear_target_encoder = 15350 + target_angle*16400;  
  move_to_target_position(2, 2, rear_target_encoder ) ; 
}


void controlRelay(uint8_t RS485_channel, uint8_t relay_number, bool state)
{
  // 모드버스 RTU 프로토콜 사용 (Function code 05)
  // Format: [Slave ID] [Function Code] [Register Address (2 bytes)] [Value (2 bytes)] [CRC (2 bytes)]
  uint8_t slave_id = 0x05;  // 기본 장치 주소 (DIP 스위치 설정에 따라 변경 가능)
  uint8_t function_code = 0x05;  // Write Single Coil
  uint8_t register_address_high = 0x00;
  uint8_t register_address_low = relay_number & 0xFF;
  uint8_t value_high = state ? 0xFF : 0x00;  // ON: 0xFF00, OFF: 0x0000
  uint8_t value_low = 0x00;
  
  // 명령 프레임 구성
  uint8_t cmd[] = 
  {
    slave_id,
    function_code,
    register_address_high,
    register_address_low,
    value_high,
    value_low
  };
  
  // CRC 계산
  uint16_t crc = CRC16_MODBUS(cmd, sizeof(cmd));
  
  // 명령 전송을 위한 전체 프레임 준비
  uint8_t full_cmd[sizeof(cmd) + 2];
  memcpy(full_cmd, cmd, sizeof(cmd));
  full_cmd[sizeof(cmd)] = crc & 0xFF;         // CRC Low byte
  full_cmd[sizeof(cmd) + 1] = (crc >> 8) & 0xFF;  // CRC High byte
  
  // RS485 채널에 따라 명령 전송
  switch(RS485_channel)
  {
    case 1: 
      RS485_Send1(full_cmd, sizeof(full_cmd));
      break;
    case 2: 
      RS485_Send2(full_cmd, sizeof(full_cmd));
      break;         
    case 3: 
      RS485_Send3(full_cmd, sizeof(full_cmd));
      break;
    default:
      if(DEBUG_ENABLE)
      {
        Serial.print("Invalid RS485 channel: ");
        Serial.println(RS485_channel);
      }
      return;
  }
  
  // 디버그 출력
  if(DEBUG_ENABLE)
  {
    Serial.print("Sent Relay Control command - Channel: ");
    Serial.print(RS485_channel);
    Serial.print(" Relay: ");
    Serial.print(relay_number);
    Serial.print(" State: ");
    Serial.print(state ? "ON" : "OFF");
    Serial.print(" - Full command: ");
    
    for(int i = 0; i < sizeof(full_cmd); i++)
    {
      printHex(full_cmd[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // 응답을 받을 시간을 주기 위한 딜레이
  delayMicroseconds(6500);  // 6.5ms의 수신 대기 시간
  
  // 응답 처리
  int bytes_read = 0;
  bool success = false;
  uint8_t response_buffer[20] = {0};
  
  // RS485 채널에 따라 응답 처리
  switch(RS485_channel)
  {
    case 1:
      while(Serial1.available() && bytes_read < 20) 
      {
        response_buffer[bytes_read++] = Serial1.read();
      }
      break;
    case 2:
      while(Serial2.available() && bytes_read < 20) 
      {
        response_buffer[bytes_read++] = Serial2.read();
      }
      break;
    case 3:
      while(Serial3.available() && bytes_read < 20) 
      {
        response_buffer[bytes_read++] = Serial3.read();
      }
      break;
  }
  
  // 응답 확인 (옵션)
  if(bytes_read >= 8) // 최소 8바이트의 응답이 와야 함
  {
    // 응답 확인 로직: 기본적으로 명령에 대한 에코가 와야 함
    if(response_buffer[0] == slave_id && 
       response_buffer[1] == function_code &&
       response_buffer[2] == register_address_high &&
       response_buffer[3] == register_address_low &&
       response_buffer[4] == value_high &&
       response_buffer[5] == value_low)
    {
      success = true;
    }
  }
  
  if(DEBUG_ENABLE)
  {
    if(bytes_read > 0)
    {
      Serial.print("Received response - bytes: ");
      Serial.print(bytes_read);
      Serial.print(" Status: ");
      Serial.println(success ? "Success" : "Failed");
      
      // 자세한 응답 내용 출력
      Serial.print("Response: ");
      for(int i = 0; i < bytes_read; i++)
      {
        printHex(response_buffer[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    else
    {
      Serial.println("No response received");
    }
  }
}

// 개별 릴레이 제어 함수들
void relayOn(uint8_t RS485_channel, uint8_t relay_number)
{
  controlRelay(RS485_channel, relay_number, true);
}

void relayOff(uint8_t RS485_channel, uint8_t relay_number)
{
  controlRelay(RS485_channel, relay_number, false);
}

// 모든 릴레이 제어 함수들
void allRelaysOn(uint8_t RS485_channel)
{
  for(uint8_t i = 0; i < 8; i++)
  {
    relayOn(RS485_channel, i);
    delay(50);  // 명령 간 짧은 지연
  }
}

void allRelaysOff(uint8_t RS485_channel)
{
  for(uint8_t i = 0; i < 8; i++)
  {
    relayOff(RS485_channel, i);
    delay(50);  // 명령 간 짧은 지연
  }
}
