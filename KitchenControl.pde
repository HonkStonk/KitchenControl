#define VOLTS_PER_UNIT (.0049F)
#define IR_SENSOR_BASE (0)
#define NUM_OF_SENSORS (5)
#define INTERVAL_50HZ  (20)
#define INTERVAL_2HZ   (100)
#define INTERVAL_5MIN  (300000)
#define LED            (13)
#define LIGHT_SWITCH   (12)
#define TRIGGER_THRESHOLD (10)

uint32_t currentMillis, previous50Hz, previous20Hz, previous2Hz, previousEvery5min;

struct sensorData {
  uint32_t sumOfSamples;
  uint8_t numberOfSamples;
  float currentDistance;
  float previousDistance;
} irSensor[NUM_OF_SENSORS];

bool atLeastOneSensorIsTriggered = false;

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  Serial.println("Initialized and ready to slay BAM");
  Serial.println(" ");
  for(int i=0; i<25; i++) {
    sampleSensors();
    delay(INTERVAL_50HZ);
  }
  readSensorsAverage();
  for(int i=0; i<NUM_OF_SENSORS; i++) {
    irSensor[i].previousDistance = irSensor[i].currentDistance; 
  }  
  digitalWrite(LED, HIGH);
}

void loop()
{      
  currentMillis = millis();
  if(currentMillis - previous50Hz > INTERVAL_50HZ) {
    previous50Hz = currentMillis;
    sampleSensors();
  }
  if(currentMillis - previous2Hz > INTERVAL_2HZ) { 
    previous2Hz = currentMillis;
    readSensorsAverage();
    if(atLeastOneSensorIsTriggered) {
      digitalWrite(LIGHT_SWITCH, HIGH);
      previousEvery5min = currentMillis; // reset 5min timer
    }
    displayHeartBeat();
  }
  if(currentMillis - previousEvery5min > INTERVAL_5MIN) {
    previousEvery5min = currentMillis;
    if(!atLeastOneSensorIsTriggered) {
      digitalWrite(LIGHT_SWITCH, LOW);
    }
  }
}
 
void sampleSensors(void) { 
    for(int i=0; i<NUM_OF_SENSORS; i++) {
      irSensor[i].sumOfSamples += analogRead(i);
      irSensor[i].numberOfSamples++;  
    }
}

void readSensorsAverage(void) {
  atLeastOneSensorIsTriggered = false; // reset trig state until at least one sensor triggers it
  for(int i=0; i<NUM_OF_SENSORS; i++) {
    irSensor[i].currentDistance = 60.495 * pow(((irSensor[i].sumOfSamples/irSensor[i].numberOfSamples)*VOLTS_PER_UNIT),-1.1904);
    irSensor[i].sumOfSamples = 0;
    irSensor[i].numberOfSamples = 0;
    Serial.print("Avg dist on sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(irSensor[i].currentDistance);
    Serial.println("cm");Serial.println("");
    if(abs(irSensor[i].previousDistance - irSensor[i].currentDistance) > TRIGGER_THRESHOLD) {
      atLeastOneSensorIsTriggered = true;      
    }
  }
}

void displayHeartBeat(void) {
    if (digitalRead(LED) == LOW) {
      digitalWrite(LED, HIGH); 
    }
    else {
      digitalWrite(LED, LOW); 
    }
}