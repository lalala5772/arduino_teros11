#include <SDISerial.h>
 
#define INPUT_SIZE 30
#define NUMSAMPLES 5
#define INVERTED 1

// 캘리브레이션 값들 (실제 측정값에 따라 조정)
#define MEASURED_MIN 6.0    // 공기 중 최소값
#define MEASURED_MAX 60.0   // 물 속 최대값
#define SCALE_MIN 0.0       // 변환된 최소값
#define SCALE_MAX 70.0      // 변환된 최대값
 
int sensorDelay = 100; 
float bulkDens = 0.4;  
float theta = 1 - (bulkDens / 2.65); 
char* samples; 
 
SDISerial sdi_serial_connection(6, INVERTED);

void setup() {
  pinMode(2, OUTPUT);
  sdi_serial_connection.begin();
  Serial.begin(9600); 
  
  // 캘리브레이션 정보 출력
  Serial.println("=== TEROS 11 Calibration Info ===");
  Serial.print("Raw range: ");
  Serial.print(MEASURED_MIN);
  Serial.print(" - ");
  Serial.println(MEASURED_MAX);
  Serial.print("Scaled range: ");
  Serial.print(SCALE_MIN);
  Serial.print(" - ");
  Serial.println(SCALE_MAX);
  Serial.println("==============================");
  Serial.println("rawValue, scaledValue, temperature, status");
}

// 측정값을 0-70 스케일로 변환하는 함수
float scaleValue(float rawValue) {
  // 선형 변환 공식
  float scaledValue = ((rawValue - MEASURED_MIN) / (MEASURED_MAX - MEASURED_MIN)) * 
                      (SCALE_MAX - SCALE_MIN) + SCALE_MIN;
  
  // 범위 제한
  if (scaledValue < SCALE_MIN) scaledValue = SCALE_MIN;
  if (scaledValue > SCALE_MAX) scaledValue = SCALE_MAX;
  
  return scaledValue;
}
 
void loop() {
  uint8_t i;
  // arrays to hold reps
  float dielectric[NUMSAMPLES];
  float soilMoist[NUMSAMPLES];
  float degC[NUMSAMPLES];
  float bulkEC[NUMSAMPLES];
  float solutionEC[NUMSAMPLES];
   
  // mean values
  float dielMean = 0.0;
  float soilMoistMean = 0.0;
  float soilMoistScaled = 0.0;  // 스케일된 값
  float degCMean = 0.0;
  float bulkECMean = 0.0;
  float solutionECMean = 0.0;
 
  // take repeated samples
  for (i = 0; i < NUMSAMPLES; i++) {
    samples = get_measurement();
    while (strlen(samples) < 5) {
      samples = get_measurement();  
    }
  
    // first term is the sensor address
    char* term1 = strtok(samples, "+");
     
    // second term is the dielectric
    term1 = strtok(NULL, "+");
    dielectric[i] = atof(term1);
               
    // third term is temperature
    term1 = strtok(NULL, "+");
    degC[i] = atof(term1);
    
    // fourth term is bulk EC
    term1 = strtok(NULL, "+");
    bulkEC[i] = atof(term1);
   
    // calculate soil moisture
    if (bulkEC[i] < 5) {
      soilMoist[i] = (5.89 * pow(10.0, -6.0) * pow(dielectric[i], 3.0)) - 
                     (7.62 * pow(10.0, -4.0) * pow(dielectric[i], 2.0)) + 
                     (3.67 * pow(10.0, -2.0) * dielectric[i]) - 
                     (7.53 * pow(10.0, -2.0));
    } else {
      soilMoist[i] = 0.118 * sqrt(dielectric[i]) - 0.117;
    }
    
    // calculate EC of solution
    solutionEC[i] = (bulkEC[i] / 1000 * soilMoist[i]) / theta; 
 
    // sum with each iteration
    dielMean += dielectric[i];
    soilMoistMean += soilMoist[i];
    degCMean += degC[i];
    bulkECMean += bulkEC[i];
    solutionECMean += solutionEC[i];
  }
   
  // Average readings for each parameter
  soilMoistMean /= NUMSAMPLES;  // 올바른 평균 계산
  soilMoistMean *= 100;  // 백분율로 변환
  degCMean /= NUMSAMPLES;
  
  // 스케일 변환 (0-70)
  soilMoistScaled = scaleValue(soilMoistMean);

  // 상태 결정 (스케일된 값 기준)
  char* status;
  if (soilMoistScaled < 21) {      // 30% of 70
    status = "low";
  }
  else if (soilMoistScaled < 49) {  // 70% of 70
    status = "good";
  }
  else {
    status = "high";
  }

  // 출력
  Serial.print(soilMoistMean, 2);    // 원본 값
  Serial.print(", ");
  Serial.print(soilMoistScaled, 1);  // 스케일된 값
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
