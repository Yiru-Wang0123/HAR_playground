/*  PulseSensor Raw Signal Test
 *  Reads raw analog signal from PulseSensor on A0.
 *  Use Arduino Serial Plotter at 9600 baud to see the waveform.
 */

int PulseSensorPin = A0;
int LED = LED_BUILTIN;
int Signal;
int Threshold = 580;

void setup() {
    pinMode(LED, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    Signal = analogRead(PulseSensorPin);
    Serial.println(Signal);

    if (Signal > Threshold) {
        digitalWrite(LED, HIGH);
    } else {
        digitalWrite(LED, LOW);
    }

    delay(20);
}
