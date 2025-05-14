#include <SDISerial.h>
 
#define INPUT_SIZE 30
#define NUMSAMPLES 5
#define INVERTED 1
#define MAX_RETRIES 10

// 캘리브레이션 값들 (실제 측정으로 조정 필요)
#define VWC_IN_AIR 0.0      // 공기 중 VWC 값 (보통 0-2%)
#define VWC_IN_WATER 50.0   // 물 속 VWC 값 (보통 40-50%)
#define SCALED_MIN 0.0      // 변환된 최소값
#define SCALED_MAX 70.0     // 변환된 최대값
 
// 전역 변수들
int sensorDelay = 100; 
float bulkDens = 0.4;  
float theta = 1 - (bulkDens / 2.65); 
char samples[100];
 
// SDI-12 통신 객체
SDISerial sdi_serial_connection(6, INVERTED);

void setup() {
  pinMode(2, OUTPUT);
  sdi_serial_connection.begin();
  Serial.begin(9600); 
  
  // CSV 헤더 출력
  Serial.println("rawVWC,scaledMoisture,degCMean,status");
  
  // 캘리브레이션 안내
  Serial.println("=== CALIBRATION INFO ===");
  Serial.println("Air calibration value: " + String(VWC_IN_AIR) + "%");
  Serial.println("Water calibration value: " + String(VWC_IN_WATER) + "%");
  Serial.println("Scaled range: 0-70");
  Serial.println("=======================");
  delay(2000);
}

// VWC 값을 0-70 스케일로 변환하는 함수
float scaleVWC(float rawVWC) {
  // 선형 변환: (x - x1) / (x2 - x1) * (y2 - y1) + y1
  float scaledValue = (rawVWC - VWC_IN_AIR) / (VWC_IN_WATER - VWC_IN_AIR) * (SCALED_MAX - SCALED_MIN) + SCALED_MIN;
  
  // 범위 제한
  scaledValue = constrain(scaledValue, SCALED_MIN, SCALED_MAX);
  
  return scaledValue;
}
 
void loop() {
  uint8_t i;
  // 센서 데이터 배열들
  float dielectric[NUMSAMPLES];
  float soilMoist[NUMSAMPLES];      // 원본 VWC 값
  float scaledMoist[NUMSAMPLES];    // 스케일된 수분값 (0-70)
  float degC[NUMSAMPLES];
  float bulkEC[NUMSAMPLES];
  float solutionEC[NUMSAMPLES];
   
  // 평균값 초기화
  float dielMean = 0.0;
  float soilMoistMean = 0.0;       // 원본 평균
  float scaledMoistMean = 0.0;     // 스케일된 평균
  float degCMean = 0.0;
  float bulkECMean = 0.0;
  float solutionECMean = 0.0;
 
  // 반복 측정
  for (i = 0; i < NUMSAMPLES; i++) {
    // 센서 데이터 읽기 (재시도 로직 포함)
    int retryCount = 0;
    do {
      get_measurement(samples, sizeof(samples));
      retryCount++;
      if(retryCount > MAX_RETRIES) {
        Serial.println("ERROR: Failed to read sensor");
        return;
      }
      delay(50);
    } while (strlen(samples) < 5);
  
    // 데이터 파싱
    char* token = strtok(samples, "+");
    
    // 유전율 읽기
    token = strtok(NULL, "+");
    if (token == NULL) continue;
    dielectric[i] = atof(token);
    
    // 온도 읽기
    token = strtok(NULL, "+");
    if (token == NULL) continue;
    degC[i] = atof(token);
    
    // Bulk EC 읽기
    token = strtok(NULL, "+");
    if (token == NULL) continue;
    bulkEC[i] = atof(token);
   
    // 토양 수분 계산 (TEROS 11 공식)
    if (bulkEC[i] < 50) {
      soilMoist[i] = (5.89e-6 * pow(dielectric[i], 3.0)) - 
                     (7.62e-4 * pow(dielectric[i], 2.0)) + 
                     (3.67e-2 * dielectric[i]) - 7.53e-2;
    } else {
      soilMoist[i] = 0.118 * sqrt(dielectric[i]) - 0.117;
    }
    
    // VWC를 백분율로 변환
    soilMoist[i] = soilMoist[i] * 100.0;
    
    // 0-70 스케일로 변환
    scaledMoist[i] = scaleVWC(soilMoist[i]);
    
    // 용액 EC 계산
    if (soilMoist[i] > 0) {
      solutionEC[i] = (bulkEC[i] / 1000.0 * soilMoist[i] / 100.0) / theta;
    } else {
      solutionEC[i] = 0;
    }
 
    // 합계 누적
    dielMean += dielectric[i];
    soilMoistMean += soilMoist[i];
    scaledMoistMean += scaledMoist[i];
    degCMean += degC[i];
    bulkECMean += bulkEC[i];
    solutionECMean += solutionEC[i];
  }
   
  // 평균 계산
  dielMean /= NUMSAMPLES;
  soilMoistMean /= NUMSAMPLES;
  scaledMoistMean /= NUMSAMPLES;
  degCMean /= NUMSAMPLES;
  bulkECMean /= NUMSAMPLES;
  solutionECMean /= NUMSAMPLES;

  // 상태 판정 (0-70 스케일 기준)
  const char* status;
  if (scaledMoistMean < 21.0) {      // 30% of 70
    status = "low";
  } else if (scaledMoistMean < 49.0) { // 70% of 70
    status = "good";
  } else {
    status = "high";
  }

  // 데이터 출력 (CSV 형식)
  Serial.print(soilMoistMean, 2);  // 원본 VWC 값
  Serial.print(",");
  Serial.print(scaledMoistMean, 1); // 스케일된 값 (0-70)
  Serial.print(",");
  Serial.print(degCMean, 1);
  Serial.print(",");
  Serial.println(status);

  delay(sensorDelay);
}

// 개선된 측정 함수
bool get_measurement(char* buffer, size_t bufferSize) {
  // SDI-12 측정 명령 전송
  char* service_request = sdi_serial_connection.sdi_query("?M!", sensorDelay);
  if (service_request == NULL) return false;
  
  // 응답 대기
  char* service_request_complete = sdi_serial_connection.wait_for_response(sensorDelay);
  if (service_request_complete == NULL) return false;
  
  // 데이터 읽기
  char* data = sdi_serial_connection.sdi_query("?D0!", sensorDelay);
  if (data == NULL) return false;
  
  // 버퍼에 복사
  strncpy(buffer, data, bufferSize - 1);
  buffer[bufferSize - 1] = '\0';
  
  return true;
}

// 캘리브레이션 모드 (필요시 사용)
void calibrationMode() {
  Serial.println("=== CALIBRATION MODE ===");
  Serial.println("1. Put sensor in air and press 'a'");
  Serial.println("2. Put sensor in water and press 'w'");
  Serial.println("3. Press 'e' to exit calibration");
  
  while(true) {
    if(Serial.available()) {
      char cmd = Serial.read();
      
      if(cmd == 'a') {
        // 공기 중 측정
        float airReading = measureRawVWC();
        Serial.print("Air reading: ");
        Serial.println(airReading);
        // 이 값을 VWC_IN_AIR로 설정
      }
      else if(cmd == 'w') {
        // 물 속 측정
        float waterReading = measureRawVWC();
        Serial.print("Water reading: ");
        Serial.println(waterReading);
        // 이 값을 VWC_IN_WATER로 설정
      }
      else if(cmd == 'e') {
        Serial.println("Exiting calibration mode");
        break;
      }
    }
    delay(100);
  }
}

// 원시 VWC 측정 함수
float measureRawVWC() {
  float sum = 0;
  int validSamples = 0;
  
  for(int i = 0; i < 10; i++) {
    get_measurement(samples, sizeof(samples));
    
    char* token = strtok(samples, "+");
    token = strtok(NULL, "+");
    if (token == NULL) continue;
    
    float dielectric = atof(token);
    float vwc = (5.89e-6 * pow(dielectric, 3.0)) - 
                (7.62e-4 * pow(dielectric, 2.0)) + 
                (3.67e-2 * dielectric) - 7.53e-2;
    vwc *= 100.0;
    
    sum += vwc;
    validSamples++;
    delay(100);
  }
  
  return sum / validSamples;
}
