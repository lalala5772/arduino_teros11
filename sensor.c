#include <SDISerial.h>
 
 
#define INPUT_SIZE 30
#define NUMSAMPLES 5
#define INVERTED 1
 
int sensorDelay = 100; 
float bulkDens = 0.4;  
float theta = 1 - (bulkDens / 2.65); 
char* samples; 
 
 
SDISerial sdi_serial_connection(6, INVERTED);


void setup() {
  pinMode(2, OUTPUT);
  sdi_serial_connection.begin();
  Serial.begin(9600); 
  Serial.println("CLEARDATA");
  Serial.println("LABEL, TIME, soilMoistMean, degCMean, status");
}
 
 
void loop() {
  
  uint8_t i;
  // arrays to hold reps
  float dielectric[NUMSAMPLES];
  float soilMoist[NUMSAMPLES];
  float degC[NUMSAMPLES];
  float bulkEC[NUMSAMPLES];
  // float porewaterEC[NUMSAMPLES];
  float solutionEC[NUMSAMPLES];
   
  // mean values
  float dielMean        = 0.0;
  float soilMoistMean   = 0.0;
  float degCMean        = 0.0;
  float bulkECMean      = 0.0;
  float solutionECMean  = 0.0;
 
  // take repeated samples
  for (i = 0; i < NUMSAMPLES; i++) {
    //char* response = get_measurement(); // get measurement data
    samples = get_measurement();
    while (strlen(samples) < 5) {
      samples = get_measurement();  
    }
  
    // first term is the sensor address (irrelevant to me)
    char* term1 = strtok(samples, "+");
     
    // second term is the dielectric conductivity & soil moisture
    term1      = strtok(NULL, "+");
    dielectric[i] = atof(term1);
               
    term1 = strtok(NULL, "+");
    degC[i] = atof(term1);
    term1 = strtok(NULL, "+");
    bulkEC[i] = atof(term1);
   
   if (bulkEC[i] < 5) {
     soilMoist[i]  = (5.89 * pow(10.0, -6.0) * pow(dielectric[i], 3.0)) - (7.62 * pow(10.0, -4.0) * 
     pow(dielectric[i], 2.0)) + (3.67 * pow(10.0, -2.0) * dielectric[i]) - (7.53 * pow(10.0, -2.0));
   } else {
   soilMoist[i]  =  0.118 * sqrt(dielectric[i]) -0.117;
   }  
    // calculate EC of solution removed from a saturated soil paste, 
    solutionEC[i] =  (bulkEC[i] / 1000 * soilMoist[i]) / theta; 
 
    // sum with each iteration
    dielMean        += dielectric[i];
    soilMoistMean   += soilMoist[i];
    degCMean        += degC[i];
    bulkECMean      += bulkEC[i];
    solutionECMean  += solutionEC[i];
  }
   
  // Average readings for each parameter
  soilMoistMean   /= (NUMSAMPLES*2000);
  degCMean        /= NUMSAMPLES;

//  Serial.print("VWC");
  Serial.print("DATA, TIME, ");
  Serial.print(soilMoistMean);
//  Serial.print("Degree");
  Serial.print(", ");
  Serial.print(degCMean, 1);
  Serial.print(", ");

  char* status;
  if(soilMoistMean < 30){
    status = "low";
    Serial.println(status);
  }
  else if(soilMoistMean < 70){
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
