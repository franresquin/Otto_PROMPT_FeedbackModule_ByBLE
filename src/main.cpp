#include <Arduino.h>
#include <M5Stack.h>
#include "Free_Fonts.h" 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "utility/CommUtil.h"

/* Visualization defines */
#define STEP_V  20
#define FRONT   4
#define X_LOCAL 60
#define Y_LOCAL 80
#define XF  30
#define YF  30

/* Unit identifiers */
#define SLAVE_ADDR        0x56
#define MOTOR_ADDR_BASE   0x00

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/* Motors*/
#define MOTORS_NUMBERS 2

/* Protocol */
#define MESSAGE_SIZE 6

#define HDR_CODE 1
#define LEFT_HAND  0
#define RIGHT_HAND 1
#define GRASP_EVENT 0
#define RELEASE_EVENT 1
#define TOUCH_EVENT 2
#define LOW_INTENSITY 160
#define MEDIUM_INTENSITY 200
#define HIGH_INTENSITY 255

#define PATTERN_LEN 25
#define SAMPLE_TIME_MS 40

byte ramp_up_low_amp[] = {120, 122, 124, 126, 129, 131, 133, 136, 138, 140, 142, 145, 147, 149, 152, 154, 156, 158, 161, 163, 165, 168, 170, 172, 175};
byte ramp_down_low_amp[] = {175, 172, 170, 168, 165, 163, 161, 158, 156, 154, 152, 149, 147, 145, 142, 140, 138, 136, 133, 131, 129, 126, 124, 122, 120};
byte ramp_up_mid_amp[] = {120, 123, 126, 130, 133, 136, 140, 143, 146, 150, 153, 156, 160, 163, 166, 170, 173, 176, 180, 183, 186, 190, 193, 196, 200};
byte ramp_down_mid_amp[] = {200, 196, 193, 190, 186, 183, 180, 176, 173, 170, 166, 163, 160, 156, 153, 150, 146, 143, 140, 136, 133, 130, 126, 123, 120};
byte ramp_up_high_amp[] = {120, 125, 131, 136, 142, 148, 153, 159, 165, 170, 176, 181, 187, 193, 198, 204, 210, 215, 221, 226, 232, 238, 243, 249, 255};
byte ramp_down_high_amp[] = {255, 249, 243, 238, 232, 226, 221, 215, 210, 204, 198, 193, 187, 181, 176, 170, 165, 159, 153, 148, 142, 136, 131, 125, 120};

BLECharacteristic *pCharacteristic;
byte count=0;

/* Motors variable definition */
bool motor1_on = false;
bool motor2_on = false;
bool motor_on[MOTORS_NUMBERS] = {false};
int16_t Speed[MOTORS_NUMBERS] = {0};
byte motor_index[MOTORS_NUMBERS] = {0};
byte motor_pattern[MOTORS_NUMBERS] = {0};
byte motor_intensity[MOTORS_NUMBERS] = {0};
CommUtil Util;

typedef enum{no_error, wrong_msg_len, undef_header, undef_hand, motor_not_on, undef_event} error_t;

/*************************************************
Function:MotorRun
Description: Motor forward and reverse API
Input:
      n: Motor 0 to motor 3
      Speed: Speed value from 0 to +255,when speed=0,The motor stopped. 
  
Return: Successful return 1
Others: 
*************************************************/
int32_t MotorRun(uint8_t mindex, int16_t speed){

    if( (mindex != 1) & (mindex != 3))
         return 0;
  
    if(speed < 0)
        speed = 0;
  
    if(speed > 255)
        speed = 255;
       
    Util.writeBytes(SLAVE_ADDR, MOTOR_ADDR_BASE+mindex*2, (uint8_t *)&speed, 2);
    
    return 1;
}

/*************************************************
Function:header
Description:The UI title
Input:
Return: 
Others: 
*************************************************/
void header(const char *string, uint16_t color){

    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLUE);
    M5.Lcd.fillRect(0, 0, 320, 30, TFT_BLUE);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(string, 160, 3, 4);
   
}

/*************************************************
Function: update_Screen
Description: display shows the intensity values of the motors
Input:
Return: 
*************************************************/
void update_screen(){

    for(byte idx=0; idx<MOTORS_NUMBERS; idx++){
        M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + YF*idx , FRONT);
        M5.Lcd.printf("M%d: %d      \n", idx, Speed[idx]);
    }

}

/* Serial port -> Check if data corresponds to the coded HEADER or TAIL */
bool is_valid_header(byte data){

  if( (data ==  HDR_CODE) )
      return true;
  else return false;

}

byte decode_intensity(byte datain){
  
  // define intensity //
  if( (datain >= 0) & (datain <= 2) ){
    Serial.println("Intensity set");
    return datain+1;
  }else{
    Serial.println("Intensity Wrong");
    return 0;
  }

}

error_t decode_Event(byte msg_data[MESSAGE_SIZE], byte motor){

  // Event? -> Grasp, release or touch //
    if( (msg_data[3] == GRASP_EVENT) ){
      // set pattern //
      Serial.println("pattern 1");
      motor_pattern[motor] = 1;
      motor_intensity[motor] = decode_intensity((byte)msg_data[5]);
      return (error_t)no_error;

    }else if(msg_data[3] == RELEASE_EVENT){
      // set pattern //
      Serial.println("pattern 2");
      motor_pattern[motor] = 2;
      motor_intensity[motor] = decode_intensity((byte)msg_data[5]);
      return (error_t)no_error;

    }else if(msg_data[3] == TOUCH_EVENT){
      // set pattern //
      Serial.println("pattern 3");
      motor_pattern[motor] = 3;
      motor_intensity[motor] = decode_intensity((byte)msg_data[5]);
      return (error_t)no_error;
      
    }else{
      // Undefine //
      Serial.println("NO pattern ");
      motor_pattern[motor] = 0;
      motor_intensity[motor] = 0;
      return (error_t)undef_event;
    }

}

/* Serial port -> DEcode to serial msg payload to take actions */
error_t decode_Serial_msg(byte msg_data[MESSAGE_SIZE]){
  
  // Left side received? //
  if( (msg_data[1] == LEFT_HAND) ){
    if( !motor_on[0] ){
      Serial.println("motor 0 -> on");
      motor_on[0] = true;
      motor_index[0] = 0;
      return decode_Event(msg_data, 0);
    }else{ 
      return (error_t)motor_not_on;
    }
  /* Left Grasp received? */
  }else if( (msg_data[1] == RIGHT_HAND) ){
    if( !motor_on[1] ){
      motor_on[1] = true;
      motor_index[0] = 0;
      Serial.println("motor 1 -> on");
      return decode_Event(msg_data, 1);
    }else {
      return (error_t)motor_not_on;
    }
  /* Undefine message, do nothing */
  }else{
    return (error_t)undef_hand;

  }

}

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      byte error_flg = 0;
      Serial.print("Message recieved: ");
      Serial.println(value.length());
      if (value.length() == MESSAGE_SIZE) {
        if( is_valid_header(value[0]) ){
          Serial.println("Decoding msg");
          error_flg = (error_t)decode_Serial_msg((byte*)&value[0]);
        }else {
          error_flg = (error_t)undef_header;
          Serial.println("Error HDR");
        }
      }else
        error_flg = (error_t)wrong_msg_len;

      //serial_feedback();
    }
};

void setup() {
  byte chr_init_data[] = {0,0,0,0,0,0};

  // put your setup code here, to run once:
  Serial.begin(115200);
  // Print init serial messages //
  Serial.println("*** Connect to M5_Device ***");

  /*Screen init*/
  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(TFT_BLACK);               
  header("PROMPT", TFT_BLACK);
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  update_screen();

  BLEDevice::init("M5_Device");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                CHARACTERISTIC_UUID,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_WRITE
                              );

  pCharacteristic->setCallbacks( new MyCallbacks() );
  pCharacteristic->setValue(chr_init_data, 6);;
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

}

byte get_motor_intensity(byte motor){

  if(motor_pattern[motor] == 1 & motor_intensity[motor] == 1){
    return ramp_up_low_amp[motor_index[motor]];
  }else if(motor_pattern[motor] == 1 & motor_intensity[motor] == 2){
    return ramp_up_mid_amp[motor_index[motor]];
  }else if(motor_pattern[motor] == 1 & motor_intensity[motor] == 3){
    return ramp_up_high_amp[motor_index[motor]];
  }else if(motor_pattern[motor] == 2 & motor_intensity[motor] == 1){
    return ramp_down_low_amp[motor_index[motor]];
  }else if(motor_pattern[motor] == 2 & motor_intensity[motor] == 2){
    return ramp_down_mid_amp[motor_index[motor]];
  }else if(motor_pattern[motor] == 2 & motor_intensity[motor] == 3){
    return ramp_down_high_amp[motor_index[motor]];
  }else if(motor_pattern[motor] == 3 & motor_intensity[motor] == 1){
    return LOW_INTENSITY;
  }else if(motor_pattern[motor] == 3 & motor_intensity[motor] == 2){
    return MEDIUM_INTENSITY;
  }else if(motor_pattern[motor] == 3 & motor_intensity[motor] == 3){
    return HIGH_INTENSITY;
  }else
    return LOW_INTENSITY;
  
}

void execute_Stim_pattern(byte motor){
  static unsigned long ttime[MOTORS_NUMBERS] = {millis()};
  byte motor_intensity_val = 0;
  byte driver_position = 0;
  unsigned long ctime = millis();

  if( motor_on[motor] ){

    if(ctime >= ttime[motor]){
      if(motor_index[motor] >= PATTERN_LEN-1){
        // halt the execution //
        motor_index[motor] = 0;
        motor_on[motor] = false;
        motor_pattern[motor] = 0;
        motor_intensity[motor] = 0;
        motor_intensity_val = 0;
      }else{
        motor_intensity_val = get_motor_intensity(motor);
        motor_index[motor]++;
      }
      if(motor == 0)
        driver_position = 1;
      else
        driver_position = 3;

      MotorRun(driver_position, motor_intensity_val);
      Speed[motor] = motor_intensity_val;
      //update_screen();
      ttime[motor] = ctime+SAMPLE_TIME_MS;

      Serial.print(".MOtor ");
      Serial.print(motor);
      Serial.print("on -> idx: ");
      Serial.print(motor_index[motor]);
      Serial.print("; int: ");
      Serial.print(motor_intensity[motor]);
      Serial.print("; pattern: ");
      Serial.print(motor_pattern[motor]);
      Serial.print("; stim: ");
      Serial.println(motor_intensity_val);
    }
  }

}
void loop() {
  // put your main code here, to run repeatedly:
  
  for(byte motor_idx=0; motor_idx<MOTORS_NUMBERS; motor_idx++){
    if( motor_on[motor_idx] ){
      execute_Stim_pattern(motor_idx);
    } 
    update_screen();
  }
  
}