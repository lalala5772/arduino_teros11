#include <SDISerial.h>
 
#define INPUT_SIZE 30
#define NUMSAMPLES 5
#define INVERTED 1
#define MAX_RETRIES 10
 
// 전역 변수들
int sensorDelay = 100; 
float bulkDens = 0.4;  
float theta = 1 - (bulkDens / 2.65); 
char samples[100]; // 고정 크기 배열로 변경
 
// SDI-12 통신 객체
SDISerial sdi_serial_connection(6, INVERTED);

void setup() {
  pinMode(2, OUTPUT);
  sdi_serial_connection.begin();
  Serial.begin(9600); 
  
  // CSV 헤더 출력
  Serial.println("soilMoistMean,degCMean,status");
}
 
void loop() {
  uint8_t i;
  // 센서 데이터 배열들
  float dielectric[NUMSAMPLES];
  float soilMoist[NUMSAMPLES];
  float degC[NUMSAMPLES];
  float bulkEC[NUMSAMPLES];
  float solutionEC[NUMSAMPLES];
   
  // 평균값 초기화
  float dielMean = 0.0;
  float soilMoistMean = 0.0;
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
    if (bulkEC[i] < 50) {  // 단위 확인 필요 (μS/cm?)
      soilMoist[i] = (5.89e-6 * pow(dielectric[i], 3.0)) - 
                     (7.62e-4 * pow(dielectric[i], 2.0)) + 
                     (3.67e-2 * dielectric[i]) - 7.53e-2;
    } else {
      soilMoist[i] = 0.118 * sqrt(dielectric[i]) - 0.117;
    }
    
    // VWC 범위 제한 (0-100%)
    soilMoist[i] = constrain(soilMoist[i], 0.0, 1.0) * 100.0;
    
    // 용액 EC 계산
    if (soilMoist[i] > 0) {
      solutionEC[i] = (bulkEC[i] / 1000.0 * soilMoist[i] / 100.0) / theta;
    } else {
      solutionEC[i] = 0;
    }
 
    // 합계 누적
    dielMean += dielectric[i];
    soilMoistMean += soilMoist[i];
    degCMean += degC[i];
    bulkECMean += bulkEC[i];
    solutionECMean += solutionEC[i];
  }
   
  // 평균 계산
  dielMean /= NUMSAMPLES;
  soilMoistMean /= NUMSAMPLES;
  degCMean /= NUMSAMPLES;
  bulkECMean /= NUMSAMPLES;
  solutionECMean /= NUMSAMPLES;

  // 상태 판정
  const char* status;
  if (soilMoistMean < 30.0) {
    status = "low";
  } else if (soilMoistMean < 70.0) {
    status = "good";
  } else {
    status = "high";
  }

  // 데이터 출력 (CSV 형식)
  Serial.print(soilMoistMean, 2);
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
