#include <Wire.h> // Arduino Uno 와 nano의 I2C 통신을 위한 라이브러리를 선언한다.
#include <Servo.h> // Servo motor를 제어하기 위한 라이브러리를 선언한다.

int A_1A = 6; // 모터를 정방향으로 fast의 속력(입력값)만큼 구동시키기 위해서 모터 드라이버의 A-1A핀에 신호를 줄 핀 번호를 "6"으로 설정한다.
int A_1B = 11; // 모터를 역방향으로 fast의 속력(입력값)만큼 구동시키기 위해서 모터 드라이버의 A-1B핀에 신호를 줄 핀 번호를 "11"으로 설정한다.
int fast = 255; // 모터의 속력(입력값)를 저장할 변수 fast를 설정한다. 또한 처음에 멈춰있는 차량을 움직이기 위해 초기값을 최대 출력인 "255"으로 설정한다.

int trigPin = 8; // 초음파 센서의 송신부(Trig)에 신호를 줄 핀 번호를 "8"로 설정한다.
int echoPin = 9; // 초음파 센서의 수신부(Echo)에 신호를 줄 핀 번호를 "9"로 설정한다.
int LED = 2; // LED를 켜기위한 핀 번호를 "2"로 설정한다.
float Distance; // 초음파 센서로 측정한 거리를 저장할 변수 Distance를 설정한다.
float Distance_pre; // 앞에서 구한 Distance 값을 저장해 놓을 변수 Distance_pre를 설정한다.

int servoPin = 10; // Servo motor에 신호를 줄 핀 번호로 "10"을 설정한다.
int angle = 90; // 차량이 정면으로 움직일 때 필요한 Servo motor의 각도 값을 저장할 변수를 angle로 설정한다.
int angle_left = 90; // 차량이 왼쪽으로 움직일 때 필요한 Servo motor의 각도 값을 저장할 변수를 angle_left로 설정한다.
int angle_right = 90; // 차량이 오른쪽으로 움직일 때 필요한 Servo motor의 각도 값을 저장할 변수를 angle_right로 설정한다.
               
Servo servo; // Servo motor를 제어할 Servo object를 servo로 설정한다.

int left = A1; // RaspberryPi3로부터 왼쪽 방향 신호를 받기 위한 핀 번호를 "A1"으로 설정한다.
int right = A2; // RaspberryPi3로부터 왼쪽 방향 신호를 받기 위한 핀 번호를 "A2"로 설정한다.
int val_left; // RaspberryPi3로부터 수신된 왼쪽 방향의 신호 값을 저장할 변수 val_left를 설정한다.
int val_right; // RaspberryPi3로부터 수신된 오른쪽 방향의 신호 값을 저장할 변수 val_right를 설정한다.

// 차량을 정면으로 움직일 forward() 함수를 설정한다.
void forward(int value) {
  analogWrite(A_1A, value); // A_1A핀의 출력 값을 함수가 선언될 때는 받은 value 값으로 설정한다.
  analogWrite(A_1B, 0); // A_1B핀의 출력 값을 0으로 한다.
}

// 차량을 정지시킬 stop() 함수를 설정한다.
void stop() {
  analogWrite(A_1A, 0); // A_1A핀의 출력 값을 0으로 한다.
  analogWrite(A_1B, 0); // A_1B핀의 출력 값을 0으로 한다.
}

// 초음파 센서를 통해 정면에 있는 장애물까지의 거리를 측정할 getDistanceCM() 함수를 설정한다.
float getDistanceCM() {
  digitalWrite(echoPin, LOW); // 처음에 echoPin핀의 출력을 LOW으로 설정한다.
  digitalWrite(trigPin, LOW); // 처음에 trigPin핀의 출력을 LOW으로 설정한다.
  delayMicroseconds(2); // 2μs를 기다린다.
  digitalWrite(trigPin, HIGH); // trigPin핀의 출력을 HIGH로 설정하여 초음파는 방사한다.
  delayMicroseconds(10); // 10μs를 기다린다.
  digitalWrite(trigPin, LOW); // trigPin핀의 출력을 LOW으로 설정하여 초음파 방사를 멈춘다.

  float distance = pulseIn(echoPin, HIGH)  / 29.0 / 2.0; // pulseIn()함수는 수신부를 활성화하여 초음파가 반사되어 돌아올 때까지의 시간을 측정한다.
                                                         // 이때 시간의 단위는 μs이고 초음파는 29μs 당 1cm를 이동한다.
                                                         // 따라서 pulseIn()함수로 측정한 시간을 29로 나눠주고 왕복이므로 한 번 더 2로 나눠주면 원하는 거리를 cm단위로 구할 수 있다.

  return distance; // 초음파 센서로 구한 장애물까지의 거리값을 반환한다.
}

// 차량 제어를 위해 필요한 초기 설정을 구성한다.
void setup() {
  Wire.begin(); //I2C 통신을 초기화하고, 활성화한다. 또한 마스터이므로 주소 값은 지정하지 않는다.

  servo.attach(servoPin); // Servo motor에 신호를 줄 핀으로 servoPin핀을 설정한다.
  servo.write(angle); // Servo motor의 각도 값(초기 값)을 angle로 설정한다.
  
  pinMode(A_1A, OUTPUT); //A_1A핀 모드를 OUTPUT으로 설정한다.
  pinMode(A_1B, OUTPUT); //A_1B핀 모드를 OUTPUT으로 설정한다.
  pinMode(trigPin, OUTPUT); //trigPin핀 모드를 OUTPUT으로 설정한다.
  pinMode(echoPin, INPUT); //echoPin핀 모드를 INPUT으로 설정한다.
  pinMode(LED, OUTPUT); //LED핀 모드를 OUTPUT으로 설정한다.

  stop(); // 차량이 처음에 정지상태에 있도록 만들어준다.

  delay(100); // 차량이 안정적으로 구동할 수 있도록 처음에 0.5초를 기다린다.
  forward(200); // 차량을 200의 속력 만큼 정면으로 구동시킨다.
                // 차량이 처음 구동할 때 관성보다 힘이 커야하기 때문에 초기 속력을 200으로 구동시킨다.
}

// 최종적으로 차량을 제어할 loop() 함수를 설정한다.
void loop() {
  Distance = getDistanceCM(); // 초음파 센서로 측정한 장애물까지의 거리를 변수 Distance에 저장한다.
  // 차량에서 장애물까지의 거리가 18cm 미만이면 차량을 멈추도록 설정한다.
  if(Distance < 18  ) {
    digitalWrite(LED, HIGH); // 차량이 멈췄음을 알려주기 위해 LED핀의 출력을 HIGH로 설정하여 LED를 킨다.
                             // 급제동 경보 시스템(ESS: Emergency Stop Signal)을 구현한 것이다.

    // 이전 루프에서 측정한 장애물까지의 거리가 18cm 이상이면 바로 이전 루프까지 차량이 움직이고 있었다는 것을 알 수 있다. 
    // 이때 바로 이전 루프까지 움직이던 차량의 관성을 억제하기 위해서 0.2초간 후진을 하도록 설정한다.
    if(Distance_pre >= 18) {
      stop(); // 차량을 멈추기 위해 모터에 아무런 출력을 주지 않는다.
      analogWrite(A_1A, 0); // A_1A핀의 출력 값을 0으로 한다.
      analogWrite(A_1B, 200); // // A_1B핀의 출력 값을 0으로 하여 차량을 살짝 후진시키므로써 움직이던 차량의 관성을 억제시킨다.
      delay(100); // 0.1초간 후진을 유지한다.
    }
    stop(); // 차량을 완전히 멈춘다.
  }

  // 차량에서 장애물까지의 거리가 18cm 이상이면 차량이 계속 움직이도록 설정한다.
  else {
    digitalWrite(LED, LOW); // 차량이 움직이고 있음을 알려주기 위해 LED핀의 출력을 LOW로 설정하여 LED를 끈다.
    Wire.requestFrom(2, 3); // "2"라는 주소 값을 가지는 슬레이브로부터 3byte의 크기의 데이터를 요청한다.
                            // 즉, PID제어를 통해 얻은 속력 값을 요청한다.

   // 슬레이브로부터 수신된 데이터의 바이트 수를 반환받는다.
   // 이때 데이터의 바이트 수를 잘 반환 받았다면 수신된 데이터를 읽는다.
    if(Wire.available()) {
      fast = Wire.read(); // 슬레이브로부터 수신된 데이터를 읽어서 변수 fast에 저장한다.
      forward(fast); // 새롭게 저장된 변수 fast 값으로 차량을 속력을 설정한다.
    }

    val_left = analogRead(left); // RaspberryPi3로부터 수신된 왼쪽 방향의 신호 값을 읽고 그 값을 변수 val_left에 저장한다.
    val_right = analogRead(right); // RaspberryPi3로부터 수신된 오른쪽 방향의 신호 값을 읽고 그 값을 변수 val_right에 저장한다.

    // RaspberryPi3로부터 수신된 값을 기준으로 차량의 회전 방향을 결정하고 이를 Servo motor로 구현한다.
    // 만약 RaspberryPi3로부터 왼쪽 방향이라고 수신된 신호 값 보다 오른쪽 방향이라고 수신된 신호 값이 충분히 더 클 경우 차량을 오른쪽으로 회전시킨다.
    if( val_left < val_right ) { 
      servo.write(angle_right); // Servo motor의 각도를 angle_right로 설정한다.
      angle_right++; // 이후 angle_right값을 1씩 증가시켜서 오른쪽으로 더 잘 회전할 수 있도록 만들어준다. 
      angle_left = 90; // 이전 루프에서 감소되었을지도 모를 angle_left값을 초기값으로 돌려준다.
    }

    // 만약 RaspberryPi3로부터 오른쪽 방향이라고 수신된 신호 값 보다 왼쪽 방향이라고 수신된 신호 값이 충분히 더 클 경우 차량을 왼쪽으로 회전시킨다.
    else if( val_left > val_right ) {
      servo.write(angle_left); // Servo motor의 각도를 angle_left로 설정한다.
      angle_left--; // 이후 angle_left값을 1씩 감소시켜서 왼쪽으로 더 잘 회전할 수 있도록 만들어준다. 
      angle_right = 90; // 이전 루프에서 증가되었을지도 모를 angle_right값을 초기값으로 돌려준다.
    }

    // 만약 RaspberryPi3로부터 오른쪽 방향이라고 수신된 신호 값과 왼쪽 방향이라고 수신된 신호 값이 비슷할 경우 차량을 정면으로 구동시킨다.
    else {
      servo.write(angle); // Servo motor의 각도를 angle로 설정한다.
      angle_left = 90; // 이전 루프에서 감소되었을지도 모를 angle_left 값을 초기값으로 돌려준다.
      angle_right = 90; // 이전 루프에서 증가되었을지도 모를 angle_right 값을 초기값으로 돌려준다.
    }
  }
  
  Distance_pre = Distance; // 앞에서 구한 Distance 값을 변수 Distance_pre에 저장한다.
}
