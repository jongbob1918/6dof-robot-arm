#include <WiFi.h>
#include <ESP32Servo.h>

// --- 1. 사용자 설정 ---
const char* ssid = "AIE_509_2.4G";
const char* password = "addinedu_class1";

// TCP 서버에 사용할 포트 번호
const int tcpPort = 23;

// --- 2. 서보 모터 설정 ---
const int NUM_SERVOS = 5;
int servoPins[NUM_SERVOS] = {23, 22, 21, 19, 26}; 
Servo servos[NUM_SERVOS];

// WiFi 서버 및 클라이언트 객체 생성
WiFiServer server(tcpPort);
WiFiClient client;

void setup() {
  Serial.begin(115200);

  // WiFi 접속
  WiFi.begin(ssid, password);
  Serial.print("\nWiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("\n\nWiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());

  // TCP 서버 시작
  server.begin();
  Serial.printf("TCP 서버 시작. 포트: %d\n", tcpPort);

  // 서보 모터 초기화
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90); 
  }
}

void loop() {
  // 새로운 클라이언트(파이썬)가 접속했는지 확인
  if (!client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("새로운 클라이언트 접속!");
    }
    return;
  }
  
  // 클라이언트가 데이터를 보냈는지 확인
  if (client.available()) {
    // \n 문자를 만날 때까지 데이터 읽기
    String command = client.readStringUntil('\n');
    
    // 명령어 파싱 및 실행 (이전과 동일)
    int commandStart = 0;
    int commandEnd = 0;
    while(command.indexOf(',', commandStart) != -1) {
      commandEnd = command.indexOf(',', commandStart);
      parseAndExecute(command.substring(commandStart, commandEnd));
      commandStart = commandEnd + 1;
    }
    parseAndExecute(command.substring(commandStart));
  }
}

void parseAndExecute(String singleCommand) {
  int colonIndex = singleCommand.indexOf(':');
  if (colonIndex != -1) {
    int servoIndex = singleCommand.substring(1, colonIndex).toInt() - 1;
    int angle = singleCommand.substring(colonIndex + 1).toInt();
    
    if (servoIndex == 2)
    { // 3번 모터는 인덱스 2
    angle = 180 - angle; // 각도를 반전
    }
  }
}