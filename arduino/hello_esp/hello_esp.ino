HardwareSerial uartSerial(1); // RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setup_uart();
  
  pinMode(PIN_ESP_STS_LED, OUTPUT);
  pinMode(PIN_AUX_20VO_EN, OUTPUT); //Aux20hello2020
  pinMode(PIN_STS_LEDS_DISABLE, OUTPUT); //Sts led
  pinMode(PIN_ROBOT_ACTIVE, OUTPUT); // Robot Active
  pinMode(PIN_LATCH, OUTPUT);//Latch
  delay(500);

  digitalWrite(PIN_LATCH, HIGH);
  digitalWrite(PIN_AUX_20VO_EN, HIGH);
  digitalWrite(PIN_LATCH, LOW);
  digitalWrite(PIN_ROBOT_ACTIVE, HIGH);


}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(PIN_ESP_STS_LED, HIGH);
  delay(500);
  digitalWrite(PIN_ESP_STS_LED, LOW);
  delay(500);
  if (uartSerial.available()) {
    int c = uartSerial.read();
    Serial.print("Received: ");
    Serial.println(c, HEX);
  }
}

void setup_uart(){
  uartSerial.begin(115200, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX); // RX, TX
//   uartSerial.onReceive(on_uart_rx);
}
