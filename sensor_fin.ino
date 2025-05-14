#include <SDISerial.h>
 
#define INPUT_SIZE 30
#define NUMSAMPLES 10  // 5에서 10으로 증가
#define INVERTED 1
#define SMOOTHING_FACTOR 0.2  // 새로운 값의 가중치 (0.0~1.0)
 
int sensorDelay = 300;  // 100ms에서 300ms로 증가
float bulkDens = 0.4;  
float theta = 1 - (bulkDens / 2.65); 
char* samples;

// 이동평균 필터를 위한 변수
float previousMappedValue = 0.0;  // 이전 값 저장
float smoothedValue = 0.0;        // 부드럽게 처리된 값
boolean firstReading = true;      // 첫 번째 측정인지 확인

// 테로스11 센서의 기존 공기 중 측정값(최소값)과 물 속 측정값(최대값) 설정
const float AIR_VALUE = 13.8;
const float WATER_VALUE = 66.5;
const float TARGET_MIN = 0.0;
const float TARGET_MAX = 70.0;
 
SDISerial sdi_serial_connection(6, INVERTED);

void setup() {
  pinMode(2, OUTPUT);
  sdi_serial_connection.begin();
  Serial.begin(9600); 
}

// 센서값을 0~70 범위로 정확하게 매핑하는 함수
float mapSensorValue(float value) {
  // 입력값이 범위를 벗어나면 경계값으로 조정
  if (value <= AIR_VALUE) return TARGET_MIN;
  if (value >= WATER_VALUE) return TARGET_MAX;
  
  // 선형 매핑 적용
  return TARGET_MIN + (value - AIR_VALUE) * (TARGET_MAX - TARGET_MIN) / (WATER_VALUE - AIR_VALUE);
}

// 이동평균 필터 적용 함수
float applySmoothing(float newValue) {
  if (firstReading) {
    smoothedValue = newValue;
    firstReading = false;
  } else {
    // 가중 이동평균: smoothedValue = (oldValue * (1-factor)) + (newValue * factor)
    smoothedValue = (smoothedValue * (1.0 - SMOOTHING_FACTOR)) + (newValue * SMOOTHING_FACTOR);
  }
  return smoothedValue;
}

// 히스테리시스 적용 함수 (선택적)
float applyHysteresis(float value, float threshold) {
  static float lastStableValue = 0.0;
  
  if (abs(value - lastStableValue) > threshold) {
    lastStableValue = value;
    return value;
  }
  return lastStableValue;
}
 
void loop() {
  uint8_t i;

  float dielectric[NUMSAMPLES];
  float soilMoist[NUMSAMPLES];
  float degC[NUMSAMPLES];
  float bulkEC[NUMSAMPLES];
  float solutionEC[NUMSAMPLES];
   
  float dielMean        = 0.0;
  float soilMoistMean   = 0.0;
  float degCMean        = 0.0;
  float bulkECMean      = 0.0;
  float solutionECMean  = 0.0;
  float result          = 0.0;
  float rawMoistValue   = 0.0;

  for (i = 0; i < NUMSAMPLES; i++) {
    samples = get_measurement();
    while (strlen(samples) < 5) {
      samples = get_measurement();  
    }
  
    char* term1 = strtok(samples, "+");

    term1 = strtok(NULL, "+");
    dielectric[i] = atof(term1);
               
    term1 = strtok(NULL, "+");
    degC[i] = atof(term1);
    term1 = strtok(NULL, "+");
    bulkEC[i] = atof(term1);
   
    if (bulkEC[i] < 5) {
      soilMoist[i] = (5.89 * pow(10.0, -6.0) * pow(dielectric[i], 3.0)) - (7.62 * pow(10.0, -4.0) * 
      pow(dielectric[i], 2.0)) + (3.67 * pow(10.0, -2.0) * dielectric[i]) - (7.53 * pow(10.0, -2.0));
    } else {
      soilMoist[i] = 0.118 * sqrt(dielectric[i]) - 0.117;
    }  
    solutionEC[i] = (bulkEC[i] / 1000 * soilMoist[i]) / theta; 
 
    // sum with each iteration
    dielMean        += dielectric[i];
    soilMoistMean   += soilMoist[i];
    degCMean        += degC[i];
    bulkECMean      += bulkEC[i];
    solutionECMean  += solutionEC[i];
  }
   
  soilMoistMean /= (NUMSAMPLES*2500);
  degCMean      /= NUMSAMPLES;

  // 원본 토양 수분 값 저장
  rawMoistValue = soilMoistMean;
  
  // 보정된 매핑 적용
  float mappedValue = mapSensorValue(rawMoistValue);
  
  // 이동평균 필터 적용으로 부드러운 변화
  float finalValue = applySmoothing(mappedValue);
  
  // 옵션: 히스테리시스 적용 (작은 변화 무시)
  // float finalValue = applyHysteresis(smoothedValue, 0.5);
  
  // 결과 출력
  Serial.print(finalValue, 1);
  Serial.print(", ");
  Serial.print(degCMean, 1);
  Serial.print(", ");

  // 상태 판정 (부드럽게 처리된 값 기준)
  char* status;
  if(finalValue < 21){    // 30% of 70
    status = "low";
    Serial.println(status);
  }
  else if(finalValue < 49){  // 70% of 70
    status = "good";
    Serial.println(status);
  }
  else {
    status = "high";
    Serial.println(status);
  }

  delay(sensorDelay);
}

char* get_measurement(){
  // function by Joran Beasley: https://github.com/joranbeasley/SDISerial/blob/master/examples/SDISerialExample/SDISerialExample.ino
  char* service_request = sdi_serial_connection.sdi_query("?M!", sensorDelay);
  //you can use the time returned above to wait for the service_request_complete
  char* service_request_complete = sdi_serial_connection.wait_for_response(sensorDelay);
  // 1 second potential wait, but response is returned as soon as it's available
  return sdi_serial_connection.sdi_query("?D0!", sensorDelay);
}
