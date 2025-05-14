#include <SDISerial.h>
 
#define INPUT_SIZE 30
#define NUMSAMPLES 5
#define INVERTED 1

// 실제 측정 범위
#define MEASURED_MIN 6.0    // 공기 중 최소값
#define MEASURED_MAX 60.0   // 물 속 최대값
 
int sensorDelay = 100; 
float bulkDens = 0.4;  
float theta = 1 - (bulkDens / 2.65); 
char* samples; 
 
SDISerial sdi_serial_connection(6, INVERTED);

void setup() {
  pinMode(2, OUTPUT);
  sdi_serial_connection.begin();
  Serial.begin(9600); 
  
  Serial.println("=== TEROS 11 Calibrated ===");
  Serial.print("Measured range: ");
  Serial.print(MEASURED_MIN);
  Serial.print(" - ");
  Serial.println(MEASURED_MAX);
  Serial.println("Output range: 0 - 70");
  Serial.println("=========================");
  delay(1000);
}

// 측정값을 0-70으로 스케일링
float scaleValue(float rawPercent) {
  // 6-60 범위를 0-70으로 매핑
  float scaled = (rawPercent - MEASURED_MIN) / (MEASURED_MAX - MEASURED_MIN) * 70.0;
  
  // 범위 제한
  if (scaled < 0) scaled = 0;
  if (scaled > 70) scaled = 70;
  
  return scaled;
}
 
void loop() {
  uint8_t i;
  float dielectric[NUMSAMPLES];
  float soilMoist[NUMSAMPLES];
  float degC[NUMSAMPLES];
  float bulkEC[NUMSAMPLES];
  float solutionEC[NUMSAMPLES];
   
  float dielMean = 0.0;
  float soilMoistMean = 0.0;
  float degCMean = 0.0;
  float bulkECMean = 0.0;
  float solutionECMean = 0.0;
 
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
      soilMoist[i] = (5.89 * pow(10.0, -6.0) * pow(dielectric[i], 3.0)) - 
                     (7.62 * pow(10.0, -4.0) * pow(dielectric[i], 2.0)) + 
                     (3.67 * pow(10.0, -2.0) * dielectric[i]) - 
                     (7.53 * pow(10.0, -2.0));
    } else {
      soilMoist[i] = 0.118 * sqrt(dielectric[i]) - 0.117;
    }
    
    solutionEC[i] = (bulkEC[i] / 1000 * soilMoist[i]) / theta; 
 
    dielMean += dielectric[i];
    soilMoistMean += soilMoist[i];
    degCMean += degC[i];
    bulkECMean += bulkEC[i];
    solutionECMean += solutionEC[i];
  }
   
  // 올바른 평균 계산
  soilMoistMean /= NUMSAMPLES;
  degCMean /= NUMSAMPLES;
  
  // VWC를 백분율로 변환
  float rawPercent = soilMoistMean * 100.0;
  
  // 6-60 범위를 0-70으로 스케일링
  float scaledPercent = scaleValue(rawPercent);

  // 상태 결정 (0-70 기준)
  char* status;
  if (scaledPercent < 21) {      // 30% of 70
    status = "low";
  }
  else if (scaledPercent < 49) {  // 70% of 70
    status = "good";
  }
  else {
    status = "high";
  }

  // 출력
  Serial.print(scaledPercent, 1);
  Serial.print(", ");
  Serial.print(degCMean, 1);
  Serial.print(", ");
  Serial.println(status);

  delay(sensorDelay);
}

char* get_measurement(){
  char* service_request = sdi_serial_connection.sdi_query("?M!", sensorDelay);
  char* service_request_complete = sdi_serial_connection.wait_for_response(sensorDelay);
  return sdi_serial_connection.sdi_query("?D0!", sensorDelay);
}
