#include <Wire.h> //Arduino Uno 와 nano의 I2C 통신을 위한 라이브러리를 선언한다.
#include <MsTimer2.h> // Timer Interrrupt를 사용하기 위한 라이브러리를 선언한다.

int HallA = 2; // 모터의 HallA 신호를 입력받을 핀의 번호를 "2"로 설정한다.
int HallB = 3; // 모터의 HallB 신호를 입력받을 핀의 번호를 "3"로 설정한다.
double c = 0; // External Interrupt 신호의 횟수를 저장할 변수 c를 지정하고 초기 값을 0으로 설정한다.
double time_now, time_pre = 0; // Arduino가 실행된 후부터 경과된 시간을 저장할 변수 time_now와 time_pre를 설정한다.
                               // 이때 time_now에는 10번 간격으로 가장 최근에 External Interrupt가 실행되었을 때의 시간 값을 저장한다.
                               // 또한 time_pre에는 바로 그 이전에 time_now에 저장된 시간 값을 저장한다.
                               // 그리고 Arduino가 처음 시작될 때는 0ms부터 시작하므로 초기 값은 둘다 0으로 설정한다.
double time_desired = 44; // External Interrupt가 10번 실행되는 동안 이상적으로 걸리길 원하는 시간 값을 저장할 변수 time_measured를 설정한다.
                          // 이상적인 시간 값을 실험을 통해 "44"로 설정한다.
double time_measured = 44; // External Interrupt가 10번 실행되는 동안 걸린 시간 값을 저장할 변수 time_measured를 설정한다.
                           // 초기 값을 이상적인 시간값인 "44"로 설정한다.
double error, error_pre = 0; // time_measured와 time_desired의 오차 값을 저장한 변수 error와 error_pre를 설정한다.
                             // 이때 error에는 가장 최근 저장된 time_measured와 time_desired의 오차 값을 저장한다.
                             // 또한 error_pre에는 바로 그 이전에 error에 저장된 오차 값을 저장한다.
                             // 초기 값을 둘 다 "0"으로 설정한다.
double P_control, I_control, D_control; // PID제어를 위해 P,I,D 값을 저장할 변수 P_control, I_control, D_control를 설정한다.
double Kp = 1.3; // P제어를 위한 게인(1.3)을 저장할 변수 Kp를 설정한다.
double Ki = 0.2; // I제어를 위한 게인(0.2)을 저장할 변수 Ki를 설정한다.
double Kd = 5.4; // D제어를 위한 게인(5.4)을 저장할 변수 Kd를 설정한다.
int PID_control = 200 ; // 최종 PID제어를 통한 결과 값(모터의 속력)을 저장할 변수 PID_control를 설정한다.
                        // 이때 초기 값은 200으로 설정한다.

// PID 제어를 위해 필요한 초기 설정을 구성한다.
void setup() {
  Wire.begin(2); //I2C 통신을 초기화하고, 활성화한다. 또한 슬레이브이므로 주소 값을 "2"로 지정한다.
  Wire.onRequest(requestEvent); // 마스터로부터 데이터를 요청 신호("Wire.requestFrom()")를 받았을 때, "requestEvent()" 함수를 실행한다.
  pinMode(HallA,INPUT); // HallA 핀을 입력모드로 설정한다.
  pinMode(HallB,INPUT); // HallB 핀을 입력모드로 설정한다.
  
  attachInterrupt(0, cnt, FALLING); // External Interrupt 0번(2번핀)에 FALLING 이 발생하면 "cnt()" 함수를 실행한다.
  MsTimer2::set(1500, myTimer); // 차량이 움직여야 될 상황에서 움직이지 못하고 멈췄을 때(ex. 경사진 곳) 최대 출력으로 경사를 오를 수 있도록 Timer Interrupt를 설정한다.
                                 // 즉, 1.5초 이상 차량이 움직이 않을 경우 "myTimer()" 함수가 실행되도록 설정한다.
}

// PID제어를 통해 차량을 일정한 속력으로 구동시킬 결과값(모터의 속력)을 구할 cnt() 함수를 설정한다.
void cnt() {
  c++; // External Interrupt가 걸릴 때마다 변수 c 값을 하나씩 증가시킨다.

  // c 값이 "10"이 될 때마다 걸린시간을 측정한다. 또한 측정한 시간값으로 PID_control 값을 구한다.
  if(c == 10) {
    time_now = millis(); // Arduino가 시작된 후부터 경과된 시간을 millis() 함수로부터 ms단위로 반환받고 그 값을 time_now에 저장한다.
    time_measured = time_now - time_pre; // 이전 시간과 현재시간의 차이값을 time_measured에 저장한다.
                                         // 즉, External Interrupt가 10번 실행되는 동안 걸린 시간값을 저장한다.
    error = time_measured - time_desired; // 이상적인 시간값과 측정된 시간값의 오차값을 저장한다.
    
    P_control = Kp*error; // P 제어(비례제어)를 위해 오차값에 P 게인을 곱한 값을 P_control에 저장한다.
                          // 하지만 오차가 작을 때는 P 제어 만으로 오차를 0으로 만들 수 없다.(정상상태의 오차를 줄이는데 한계가 있다.)
    I_control += Ki*error*time_measured; // I 제어(적분제어)를 위해 과거부터 현재까지 생긴 오차를 누적한다.
                                         // 오차값에 측정된 시간값과 I 게인을 곱한 값을 계속 누적하고 그 값을 I_control에 저장한다.
                                         // 즉, I제어를 통해 정상상태의 오차를 줄인다.(P 제어의 문제점을 보완한다.)
    D_control = Kd*((error-error_pre)/(time_measured)); // D 제어(미분제어)를 위해 측정된 시간에 따른 오차값의 차이를 구한다.
                                                        // 이전 오차값과 현재 오차 값의 차이를 측정된 시간 값으로 나누고 이를 D 게인과 곱한 값을 D_control에 저장한다. 
                                                        // 즉, D제어를 통해 반응이 더욱 부드러워진다.

    PID_control = 200 + P_control + I_control + D_control; // 초기 값인 200에 앞에서 구한 P,I,D 제어의 결과 값에 더하여 PID_control 값을 구한다.
    PID_control = constrain(PID_control, 0, 255); // PWM 출력에 맞게 PID_control 값을 0이상 255이하로 제한한다.
    c = 0; // 다시 External Interrupt가 10번 실행되었을 때를 구하기 위해 변수 c의 값을 0으로 초기화시킨다.
    
    error_pre = error; // 앞에서 구한 error 값을 변수 error_pre에 저장한다.
    time_pre = time_now; // 앞에서 구한 time_now 값을 변수 time_pre에 저장한다.
  }
  MsTimer2::start(); // Timer Interrupt가 실행되기 위한 대시 시간을 다시 0ms로 만들어 준다.
                      // 즉, External Interrupt가 제대로 작동되고 있다면(차량이 계속 움직이고 있다면) Timer Interrupt가 실행되지 않도록 만들어준다.
}

// 차량의 출력을 최대로 만들어줄 myTimer() 함수를 설정한다..
void myTimer()
{
  PID_control = 255; // PID_control 변수에 255를 저장한다.
}

// 마스터에게 PID_control값을 보내줄 requestEvent() 함수를 설정한다. 
void requestEvent()
{
  Wire.write(PID_control); // 마스터로 "PID_control" 값을 보내준다.
}

// loop()함수는 비워둔다.
void loop() {

}
