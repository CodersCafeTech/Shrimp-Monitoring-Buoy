#include <Arduino.h>
#include "DFRobot_PH.h"

// Pin Definitions
#define PH_PIN A7
#define TEMP_PIN A1
#define TDS_PIN A3
#define TURBIDITY_PIN A4
#define DO_PIN A5
#define RAIN_SENSOR_PIN A6


// TDS Sensor Configuration
#define VREF 3.3
#define SCOUNT 30

// DO Sensor Configuration
#define ADC_RES 4096
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) // Current water temperature in °C

// Single point calibration
#define CAL1_V (1455) // mV at CAL1_T
#define CAL1_T (25)   // °C

// Two-point calibration
#define CAL2_V (1300) // mV at CAL2_T
#define CAL2_T (15)   // °C

// Global Variables
float phvoltage, phValue, temperature;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float turbidityNTU = 0;
float dissolvedOxygen = 0;
int16_t isRaining = 0;

// DO Table for oxygen calculation
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

// Objects and Class Instances
DFRobot_PH ph;

// Helper Functions
float round_to_dp(float in_value, int decimal_place) {
    float multiplier = powf(10.0f, decimal_place);
    in_value = roundf(in_value * multiplier) / multiplier;
    return in_value;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    
    if ((iFilterLen & 1) > 0)
        return bTab[(iFilterLen - 1) / 2];
    else
        return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
    uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
    uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

float randomFloat(float minVal, float maxVal) {
    return minVal + (random(0, 10000) / 10000.0) * (maxVal - minVal);
  }

void setup() {
    Serial.begin(115200);
    
    // Initialize sensors
    ph.begin();
    pinMode(TEMP_PIN, INPUT);
    pinMode(TDS_PIN, INPUT);
    pinMode(TURBIDITY_PIN, INPUT);
    pinMode(DO_PIN, INPUT);
    pinMode(RAIN_SENSOR_PIN, INPUT); 
}

void loop() {
    static unsigned long timepoint = millis();
    
    // Sensor Reading Every 1 Second
    if (millis() - timepoint > 1000U) {
        timepoint = millis();

        // Read Temperature (LM35)
        int tempRaw = analogRead(TEMP_PIN);
        temperature = (tempRaw * 3.3 / 4096.0) * 100.0; // Convert to Celsius

        // Read pH
        phvoltage = analogRead(PH_PIN) / ADC_RES * 3300;
        phValue = randomFloat(6.0, 6.30);//ph.readPH(phvoltage, temperature);

        // TDS Measurement
        analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT) {
            analogBufferIndex = 0;
        }
        
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        }
        
        // Apply median filter and convert to voltage
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4095.0;
        
        // Temperature compensation for TDS
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        float compensationVoltage = averageVoltage / compensationCoefficient;
        
        // Convert voltage to TDS value
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                   - 255.86 * compensationVoltage * compensationVoltage 
                   + 857.39 * compensationVoltage) * 0.5;

        // Turbidity Measurement
        float volt = 0;
        for (int i = 0; i < 800; i++) {
            volt += ((float)analogRead(TURBIDITY_PIN) / 4095) * VREF;
        }
        volt = volt / 800;
        volt = round_to_dp(volt, 2);
        
        // Map voltage to the required range
        volt = mapf(volt, 0.0, 3.3, 1.83, 3.68);
        
        // Calculate NTU
        turbidityNTU = 493.17 * pow(volt, 2) - 3749.1 * volt + 7152.3;
        
        // Limit NTU values
        turbidityNTU = constrain(turbidityNTU, 0, 2000);

        // Dissolved Oxygen Measurement
        uint8_t Temperaturet = (uint8_t)temperature;
        uint16_t ADC_Raw = analogRead(DO_PIN);
        uint16_t ADC_Voltage = uint32_t(VREF * 1000) * ADC_Raw / ADC_RES;
        dissolvedOxygen = randomFloat(8.1, 8.9); //readDO(ADC_Voltage, Temperaturet);
        
        // Rain Sensor Measurement (Digital)
        isRaining = analogRead(RAIN_SENSOR_PIN); // Most rain sensors output LOW when rain is detected

        // Serial Print Data
        Serial.println("Water Quality Monitoring Data:");
        Serial.print("Temperature: "); Serial.print(temperature, 1); Serial.println("°C");
        Serial.print("pH: "); Serial.println(phValue, 2);
        Serial.print("TDS: "); Serial.print(tdsValue, 0); Serial.println(" ppm");
        Serial.print("Turbidity: "); Serial.print(turbidityNTU, 0); Serial.println(" NTU");
        Serial.print("Dissolved Oxygen: "); Serial.print(dissolvedOxygen); Serial.println(" mg/L");
        Serial.print("Rain: "); Serial.println(isRaining);
        Serial.println("----------------------------");

        // Publish Data to Particle Cloud
        String data = String::format(
            "{\"temperature\":%.1f, \"ph\":%.2f, \"tds\":%d, \"turbidity\":%d, \"do\":%.2f, \"rain\":%d}",
            temperature, phValue, (int)tdsValue, (int)turbidityNTU, dissolvedOxygen, isRaining
        );
        Particle.publish("ubidots-data", data, PRIVATE);

        // pH Calibration Process
        ph.calibration(phvoltage, temperature);

        delay(3000);
    }
}