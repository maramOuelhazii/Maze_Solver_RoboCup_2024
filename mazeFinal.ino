#include <TimerOne.h>
#include <Servo.h>
#include "Wire.h"
#include "Adafruit_TCS34725.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

Servo servo;//45
//Servo servo; //43

TimerOne timer;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int sharpR = A11;
int sharpL = A10;
int sharpF = A9;
int sharpB = A8;

int led = 41;


int sharpFRDB[4] = {sharpF, sharpR, sharpB, sharpL};

bool storingCheckpoint = false;
int pushButtonPin = 35;

int orientation_gyro = 0;

struct Coordinates
{
  int x;
  int y;
};

struct FiveBitValue
{
  byte value : 5;
};

const int matrixSize = 70;
FiveBitValue lastCheckpoint[matrixSize][matrixSize];

const int maxSize = 50;
Coordinates pile[maxSize];
int pileSize = 0;
void addToPile(int x, int y)
{
  if (pileSize < maxSize)
  {
    pile[pileSize].x = x;
    pile[pileSize].y = y;
    pileSize++;
  }
  else
  {
    Serial.println("Pile is full");
  }
}

void printPile()
{
  for (int i = 0; i < pileSize; i++)
  {
    Serial.print("Coordinates ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(pile[i].x);
    Serial.print(", ");
    Serial.print(pile[i].y);
    Serial.println(")");
  }
}

void removeLastFromPile()
{
  if (pileSize > 0)
  {
    pileSize--;
  }
  else
  {
    Serial.println("Pile is empty");
  }
}


FiveBitValue maze[matrixSize][matrixSize];

void initializeMatrix()
{
  for (int i = 0; i < matrixSize; i++)
  {
    for (int j = 0; j < matrixSize; j++)
    {
      maze[i][j].value = 0b0; // Set each element to 0
    }
  }
}

int current_x = matrixSize / 2;
int current_y = matrixSize / 2;
int last_8_x = current_x;
int last_8_y = current_y;
int vitesse = 170;

int finCourseD = 52;
int finCourseG = 53;

int right_pinA = 19;//3;1
int right_pinB = 18;//2;

int left_pinA = 3;//18;
int left_pinB = 2;//19;

int right_motorF = 6;
int right_motorB = 7;

int left_motorF = 4;
int left_motorB = 5;

int pinBlack = 51;

volatile long int current_right_ticks = 0;
volatile long int current_left_ticks = 0;
volatile long int previous_right_ticks = 0;
volatile long int previous_left_ticks = 0;
volatile long int d_right_ticks = 0;
volatile long int d_left_ticks = 0;

volatile long int right_ticks = 0;
volatile long int left_ticks = 0;

volatile float orientation_rad = 0;
volatile float orientation_deg = 0;

volatile float dR = 0;
volatile float dL = 0;
volatile float dC = 0;
volatile float dTheta = 0;

volatile float dR_total = 0;
volatile float dL_total = 0;
volatile float dC_total = 0;

volatile float dR_speed = 0;
volatile float dL_speed = 0;
volatile float dTheta_speed = 0;

volatile float dX = 0;
volatile float dY = 0;

volatile float X = 0;
volatile float Y = 0;

volatile float right_encoder_speed = 0;
volatile float left_encoder_speed = 0;
volatile float theta_encoder_speed = 0;

float PWM_R = 0;
float PWM_L = 0;
float PWM_MIN = 80;
float PWM_MAX = 255;

float right_erreur = 0;
float left_erreur = 0;
float i_right_erreur = 0;
float i_left_erreur = 0;

float orientation = 0;
float i_orientation = 0;
float orientation_erreur = 0;
float i_orientation_erreur = 0;
float Theta_correction = 0;
float position_erreur = 0;

float kp = 60;//50
float kp_dour = 40;
float ki = 0;
float kTheta = 80;
float k_position = 50;

int sens = 1;

volatile long int t = 0;
int T = 10;
int precision = 4;
float resoultion = 700;

float radius_r = 45;//47
float radius_l = 45.00;//43

float entreAxe = 320;

float rotation = 0;

int sharp_7it = 20;

String state;

void setBitOn(FiveBitValue &data, int position)
{
  data.value |= (1 << position);
}
void setBitOff(FiveBitValue &data, int position)
{
  data.value &= ~(1 << position);
}

bool getBit(const FiveBitValue &data, int position)
{
  return (data.value >> position) & 1;
}
int distance(int pin)
{Serial.println("i am in distance");
  int moy = 0;
  for (int i = 0; i < 20; i++)
  {
    float volts = analogRead(pin) * 0.0048828125;
    float distance = 13 * pow(volts, -1);
    moy = distance + moy;
  }
  return moy / 20;
}

//int distance(int pin)()
//{
//  int moy = 0;
//  for (int i = 0; i < 20; i++)
//  {
//    float volts = analogRead(sharpF) * 0.0048828125;
//    float distance = 13 * pow(volts, -1);
//    moy = distance + moy;
//  }
//  return moy / 20;
//}
//int distance(int pin)()
//{
//  int moy = 0;
//  for (int i = 0; i < 20; i++)
//  {
//    float volts = analogRead(sharpR) * 0.0048828125;
//    float distance = 13 * pow(volts, -1);
//    moy = distance + moy;
//  }
//  return moy / 20;
//}int distance(int pin)()
//{
//  int moy = 0;
//  for (int i = 0; i < 20; i++)
//  {
//    float volts = analogRead(sharpB) * 0.0048828125;
//    float distance = 13 * pow(volts, -1);
//    moy = distance + moy;
//  }
//  return moy / 20;
//}int distance(int pin)()
//{
//  int moy = 0;
//  for (int i = 0; i < 20; i++)
//  {
//    float volts = analogRead(sharpL) * 0.0048828125;
//    float distance = 13 * pow(volts, -1);
//    moy = distance + moy;
//  }
//  return moy / 20;
//}


void showMaze()
{

  for (int i = 0; i < matrixSize; i++)
  {
    for (int j = 0; j < matrixSize; j++)
    {
      Serial.print(maze[i][j].value);
      Serial.print(" "); // Add a tab to separate elements
    }
    Serial.println(); //  to the next row
  }
}
void stoop()
{
  analogWrite(right_motorF, LOW);
  analogWrite(right_motorB, LOW);
  analogWrite(left_motorF, LOW);
  analogWrite(left_motorB, LOW);
}

void forward()
{
  analogWrite(right_motorF, vitesse);
  analogWrite(right_motorB, LOW);
  analogWrite(left_motorF, vitesse);
  analogWrite(left_motorB, LOW);
}
void forwardPorMax(int vit)
{
  analogWrite(right_motorF, vit);
  analogWrite(right_motorB, LOW);
  analogWrite(left_motorF, vit);
  analogWrite(left_motorB, LOW);
}
void backward()
{
  analogWrite(right_motorF, LOW);
  analogWrite(right_motorB, vitesse);
  analogWrite(left_motorF, LOW);
  analogWrite(left_motorB, vitesse);
}

void right()
{
  analogWrite(right_motorF, vitesse);
  analogWrite(right_motorB, LOW);
  analogWrite(left_motorF, LOW);
  analogWrite(left_motorB, vitesse);
}

void left()
{
  analogWrite(right_motorF, LOW);
  analogWrite(right_motorB, vitesse);
  analogWrite(left_motorF, vitesse);
  analogWrite(left_motorB, LOW);
}

String check(int x, int y)
{Serial.println("i am in check");
  setBitOn(maze[current_x][current_y], 4);
  showMaze();
  String second_decision = "";
  bool up = false;
  bool right = false;
  bool down = false;
  bool left = false;
  int dist_up;
  int dist_right;
  int dist_down;
  int dist_left;

  int dx[4] = { -1, 0, 1, 0};
  int dy[4] = {0, 1, 0, -1};

  int orientations[4] = {0, -90, 180, 90};
  int dist[4];

  for (int i = 0; i < 4; ++i)
  {
    int dir = (4 + orientation_gyro / 90 + i) % 4;
    dist[i] = distance(sharpFRDB[dir]);
  }
  //int dir = (4 + orientation_gyro / 90 + 1) % 4;
  //dist[0]=distance(int pin)();
  // dir = (4 + orientation_gyro / 90 + 2) % 4;
  //dist[1]=distance(int pin)();
  // dir = (4 + orientation_gyro / 90 + 3) % 4;
  //dist[2]=distance(int pin)();
  // dir = (4 + orientation_gyro / 90 + 4) % 4;
  //dist[3]=distance(int pin)();

  last_8_x = pile[pileSize - 1].x;
  last_8_y = pile[pileSize - 1].y;

  for (int i = 0; i < 4; ++i)
  {
    int new_x = x + dx[i];
    int new_y = y + dy[i];

    if (dist[i] < sharp_7it)
    {
      if (!getBit(maze[new_x][new_y], 4))
      {
        setBitOn(maze[new_x][new_y], (i + 2) % 4);
      }
    }
    else
    {
      if (!getBit(maze[new_x][new_y], 4) && !getBit(maze[new_x][new_y], (i + 2) % 4))
      {
        switch (i)
        {
          case 0:
            up = true;
            break;
          case 1:
            right = true;
            break;
          case 2:
            down = true;
            break;
          case 3:
            left = true;
            break;
        }
      }
      else if (getBit(maze[new_x][new_y], 4) && last_8_x == new_x && last_8_y == new_y)
      {
        switch (i)
        {
          case 0:
            second_decision = "up";
            break;
          case 1:
            second_decision = "right";
            break;
          case 2:
            second_decision = "down";
            break;
          case 3:
            second_decision = "left";
            break;
        }
      }
    }
  }

  setBitOn(maze[x][y], 4);

  String last_decision = "";
  if (up || right || down || left)
  {
    if (up)
      last_decision = "up";
    else if (right)
      last_decision = "right";
    else if (down)
      last_decision = "down";
    else if (left)
      last_decision = "left";
    addToPile(x, y);
  }
  else
  {
    last_decision = second_decision;
    //maze[x][y] = '1';
    removeLastFromPile();
  }

  if (distance(sharpF) < sharp_7it)
  {
    while (digitalRead(finCourseG) || digitalRead(finCourseD))
    {
      Serial.print(digitalRead(finCourseG));
      Serial.println(digitalRead(finCourseD));
      forward();
    }
    stoop();
    move(-150, 100);
  }

  return last_decision;
}

float rad2Deg(float rad)
{
  return rad * 180 / PI;
}

void iniiit()
{Serial.println("i am in iniiit ");
  dR_total = 0;
  
  dL_total = 0;
  dC_total = 0;
  i_right_erreur = 0;
  i_left_erreur = 0;
  right_erreur = 0;
  left_erreur = 0;
  position_erreur = 0;
  orientation_erreur = 0;
}

float erreur(float PWM, float min, float max)
{
  if (PWM < min)
  {
    PWM = min;
  }
  else if (PWM > max)
  {
    PWM = max;
  }
  return PWM;
}

void run()
{//Serial.println("i am in run ");

  if (PWM_R > 0)
  {
    // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
    analogWrite(right_motorF, PWM_R);
    analogWrite(right_motorB, 0);
  }
  else
  {
    // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
    analogWrite(right_motorF, 0);
    analogWrite(right_motorB, -PWM_R);
  }
  if (PWM_L > 0)
  {
    // PWM_L=erreur(PWM_L,PWM_MIN,PWM_MAX);

    analogWrite(left_motorF, PWM_L);
    analogWrite(left_motorB, 0);
  }
  else
  {
    // PWM_L=erreur(PWM_L,-PWM_MAX,-PWM_MIN);

    analogWrite(left_motorF, 0);
    analogWrite(left_motorB, -PWM_L);
  } if (PWM_L > PWM_R)
  {
    // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
    analogWrite(right_motorF, PWM_R);
    analogWrite(right_motorB, 0);
    analogWrite(left_motorF, PWM_L - 40);
    analogWrite(left_motorB, 0);
  }

}
void STOP()
{
  analogWrite(right_motorF, 0);
  analogWrite(right_motorB, 0);
  analogWrite(left_motorF, 0);
  analogWrite(left_motorB, 0);
}
float ticks2Distance(long int ticks, float radius, float resolution, int precision)
{
  return ticks * 2 * PI * radius / (resolution * precision);
}
float acceleration(float speed, float distance, float accel, float decel)
{
  float current_speed;

  if (abs(dC_total) < accel)
  {
    current_speed = (speed / (accel)) * abs(dC_total);
  }
  else if (distance - abs(dC_total) < decel)
  {
    current_speed = (speed / -decel) * abs(dC_total) + speed - ((distance - decel) * (speed / -decel));
  }
  else
  {
    current_speed = speed;
  }
  return current_speed;
}

float acceleration_dour(float speed, float distance, float accel, float decel)
{
  float current_speed;

  if ((dR_total - dL_total) < accel)
  {
    current_speed = (speed / (accel)) * (dR_total - dL_total);
  }
  else if (distance - (dR_total - dL_total) < decel)
  {
    current_speed = (speed / -decel) * (dR_total - dL_total) + speed - ((distance - decel) * (speed / -decel));
  }
  else
  {
    current_speed = speed;
  }

  // current_speed = erreur(current_speed, PWM_MIN, speed);
  return current_speed;
}

void dour(int angle)
{Serial.println("i am in dour ");
  orientation_gyro += angle;
  mpu.update();
  int zero = mpu.getAngleZ();

  while (abs(mpu.getAngleZ() - (zero + angle)) > 0.01)
  {

    mpu.update();

    if (mpu.getAngleZ() - (zero + angle) < 0)
    {

      left();
    }
    else
    {
      right();
    }
  }
  stoop();
}
bool sa3da() {
  Serial.println("i am in sa3da ");
  mpu.update();
  if (mpu.getAngleY() > 15) {
    return true;
  }
  else return false ;
}
void move(float distance, float speed)
{ //Serial.print("we are in move ");
  iniiit();
  float accel = 0.25 * distance;
  float decel = 0.5 * distance;

  while (abs(dC_total - distance) > 5)
 { //Serial.print(dL_total);
//  Serial.print("/        /");
//    Serial.println(dR_total);
    
    if ((dC_total - distance) < 0)
      sens = 1;
    else
      sens = -1;

    float current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));
    // right pid
    right_erreur = current_speed + right_encoder_speed;
    i_right_erreur = right_erreur;
    PWM_R = kp * right_erreur + ki * i_right_erreur;
    if (sens == 1)
    {
      PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
    }
    else
    {
      PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
    }
    // left pid
    left_erreur = current_speed - left_encoder_speed;
    i_left_erreur += left_erreur;
    PWM_L = kp * left_erreur + ki * i_left_erreur;

    if (sens == 1)
    {
      PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);

    }
    else
    {
      PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
    }
    // orientation pid
    orientation_erreur = dR_total - dL_total;
    Theta_correction = kTheta * orientation_erreur;// ktheta lazim nzid faha bech yitisstwa robot

    PWM_R += Theta_correction;
    PWM_L -= Theta_correction;
    if (sens == 1)
    {
      PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
    }
    else
    {
      PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
    }
    if (sens == 1)
    {
      PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
    }
    else
    {
      PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
    }
    //Serial.println("ruuuuuuuuuuuun");
    run();
  }
  //Serial.println("stoooooooooooooop");
  STOP();
  delay(4000);
}

void orienter(float orientation)
{Serial.println("i am in orienter");
  int target_angle = orientation - orientation_gyro;

  while (target_angle > 90)
  {
    target_angle -= 360;
  }

  while (target_angle < -90)
  {
    target_angle += 360;
  }

  dour(target_angle);
}

void doEncodeA0()
{
  if (digitalRead(right_pinA) != digitalRead(right_pinB))
  {
    right_ticks++;
  }
  else
  {
    right_ticks--;
  }
}
void doEncodeB0()
{
  if (digitalRead(right_pinA) == digitalRead(right_pinB))
  {
    right_ticks++;
  }
  else
  {
    right_ticks--;
  }
}
void doEncodeA1()
{
  if (digitalRead(left_pinA) != digitalRead(left_pinB))
  {
    left_ticks++;
  }
  else
  {
    left_ticks--;
  }
}
void doEncodeB1()
{
  if (digitalRead(left_pinA) == digitalRead(left_pinB))
  {
    left_ticks++;
  }
  else
  {
    left_ticks--;
  }
}

void updatePosition()
{
//Serial.println("i am in updatePosition");
  previous_right_ticks = current_right_ticks;
  previous_left_ticks = current_left_ticks;
  current_right_ticks = right_ticks;
  current_left_ticks = left_ticks;

  d_right_ticks = current_right_ticks - previous_right_ticks;
  d_left_ticks = current_left_ticks - previous_left_ticks;

  dR = ticks2Distance(d_right_ticks, radius_r, resoultion, precision);
  dL = ticks2Distance(d_left_ticks, radius_l, resoultion, precision);

  dR_total += dR;
  dR_speed += dR;
  dL_total += dL;
  dL_speed += dL;

  dC = (dR + dL) / 2;
  dC_total += dC;

  dTheta = (dR - dL) / entreAxe;

  dL_speed += dTheta;
  orientation_rad += dTheta;

  while (orientation_rad > PI)
  {
    orientation_rad -= 2 * PI;
  }

  while (orientation_rad < -PI)
  {
    orientation_rad += 2 * PI;
  }

  orientation_deg = rad2Deg(orientation_rad);

  dX = dC * cos(orientation_rad);
  dY = dC * sin(orientation_rad);

  X += dX;
  Y += dY;
     Serial.print(dL_total);
      Serial.print("      ");
      Serial.print(dR_total);
      Serial.print("      ");
      Serial.println(dC_total);
}

void updateSpeed()
{
//Serial.println("i am in updateSpeed ");
  right_encoder_speed = dR_speed * 1000 / (T * 5);
  dR_speed = 0;

  left_encoder_speed = dL_speed * 1000 / (T * 5);
  dL_speed = 0;

  theta_encoder_speed = dTheta_speed * 1000 / (T * 5);
  dTheta_speed = 0;
}

void callback()
{//Serial.println("i am in callback ");
  t++;
  updatePosition();

  if (t % T == 0)
  {
    updateSpeed();
  }
}
void hole() {
  Serial.println("i am in hole");
  current_x = current_x + 1;
  //current_y = current_y
  setBitOn(maze[current_x][current_y], 4);
  orienter(180);
  move(110, vitesse);
  orienter(180);
}
bool detectBlack() {
  if ( digitalRead(pinBlack)) {
    return "black";
  }
}
void storeCheckpoint()
{Serial.println("i am in  storeCheckpoiont");
  for (int i = 0; i < matrixSize; i++)
  {
    for (int j = 0; j < matrixSize; j++)
    {
      lastCheckpoint[i][j].value = maze[i][j].value;
    }
  }
}
void checkpoint() {
  Serial.println("i am in checkpoint");
  state = color();

  if (storingCheckpoint)
  {
    if (state == "silver" || digitalRead(pushButtonPin) == LOW)
    {
      storingCheckpoint = false;
    }
  }
  else
  {
    if (state == "silver")
    {
      storingCheckpoint = true;
      storeCheckpoint();
    }
  }
  }
  String color() {
    Serial.println("i am in color");
    
    float red, green, blue;
  
    tcs.setInterrupt(false);  // turn on LED
  
    delay(60);  // takes 50ms to read
  
    tcs.getRGB(&red, &green, &blue);
  
    tcs.setInterrupt(true);  // turn off LED
  
    //Serial.print("R:\t"); //Serial.print(int(red));
    //Serial.print("\tG:\t"); //Serial.print(int(green));
    //Serial.print("\tB:\t");// Serial.print(int(blue));
    int r, g, b;
    r = int (red);
    g = int (green);
    b = int (blue);
    if ((b >= 70 && b <= 80) && (r >= 80 && r <= 90) && (g >= 90 && g < 100)) {
      //Serial.println ("silvre");
      //checkpoint();
      //state="silver";
      return "silver ";
    }
    else if (b >= 110 && r <= 63 && g < 100) {
      //Serial.println ("blue");
      //stoop();
      //delay(5000);
      return "blue";
    } else if (detectBlack()) {
      return "black";
    }
    else return "noColor";
  
    //Serial.print("\n");
  }
void go() {
  emchi(300);
//  state = color();
//  if (state == "blue") {
//    STOP();
//    delay(5000);
//  }
//  else if (state == "silver") {
//    checkpoint();
//  }
//  else if (state == "black") {
//    hole();
//  }
//  while (sa3da()) {
//    forwardPorMax(255);
//  }
}
void emchi(int vitesse)
{
  Serial.println("i am in emchi");
//    Serial.print(distance(sharpF));
//    Serial.print("    ");
//    Serial.print(distance(sharpR));
//    Serial.print("    ");
//    Serial.print(distance(sharpB));
//    Serial.print("    ");
//    Serial.println(distance(sharpL));
//  while (true)
//  {
//    if (Serial.available() > 0)
//    {
//      if (Serial.read() == 'c')
//      {
//        break;
//      }
//    }
//  }

  String dic = check(current_x, current_y);
  //Serial.print(dic);
//  while (true)
//  {
//    if (Serial.available() > 0)
//    {
//      if (Serial.read() == 'c')
//      {
//        break;
//      }
//    }
//  }
  if (dic == "up")
  {
    current_x = current_x - 1;
  }
  else if (dic == "right")
  {
    current_y = current_y + 1;
  }
  else if (dic == "down")
  {
    current_x = current_x + 1;
  }
  else if (dic == "left")
  {
    current_y = current_y - 1;
  }

  if (dic == "up")
  {
    move(300, vitesse);
  }
  if (dic == "right")
  {
    orienter(90);
    move(300, vitesse);
  }
  if (dic == "down")
  {
    orienter(180);
    move(300, vitesse);
  }
  if (dic == "left")
  {
    orienter(-90);
    move(300, vitesse);
  }
}
void setup()
{

  Serial.begin(115200);
  Serial.println("Startingggggg ");
  timer.initialize(5000);
  timer.attachInterrupt(callback);
  Serial.println("call");
  pinMode(led, INPUT);

  // pin encodeur
  pinMode(right_pinA, INPUT);
  pinMode(right_pinB, INPUT);
  pinMode(left_pinA, INPUT);
  pinMode(left_pinA, INPUT);

  // pin sharp
  pinMode(sharpL, INPUT);
  pinMode(sharpR, INPUT);
  pinMode(sharpF, INPUT);
  pinMode(sharpB, INPUT);

  // Motor pins setup
  pinMode(right_motorF, OUTPUT);
  pinMode(right_motorB, OUTPUT);
  pinMode(left_motorF, OUTPUT);
  pinMode(left_motorB, OUTPUT);

  // pin finCourse
  pinMode(finCourseD, INPUT_PULLUP);
  pinMode(finCourseG, INPUT_PULLUP);

  pinMode(pinBlack, INPUT);
  pinMode(pushButtonPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_pinA), doEncodeA0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_pinB), doEncodeB0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_pinA), doEncodeA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_pinB), doEncodeB1, CHANGE);

  // analogWrite(right_motorF, vitesse);
  //   analogWrite(right_motorB, LOW);
  //   analogWrite(left_motorF, vitesse);
  //   analogWrite(left_motorB, LOW);
  if (tcs.begin()) {

  } else {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1);
  //}
  Serial.println("TCS3472555555555555555555555555555555555");
  byte status = mpu.begin();
  Serial.println("mpuuuuuuuuuuuuuuuuuuuuuuuuuuuu");
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  //while (status != 0)
  //{
  //}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // timerG=millis();
  // now=gyro.getAngleZ();

  //    while(true){
  //      Serial.print(distance(int pin)());
  //      Serial.print("    ");
  //      Serial.print(distance(int pin)());
  //      Serial.print("    ");
  //      Serial.print(distance(int pin)());
  //      Serial.print("    ");
  //      Serial.println(distance(int pin)());
  //    }

  //    while(true){
  //      Serial.print(digitalRead(finCourseG));
  //      Serial.println(digitalRead(finCourseD));
  //    }
  //  stoop();
  initializeMatrix();
  int middleRow = matrixSize / 2;
  int middleColumn = matrixSize / 2;

  setBitOn(maze[middleRow][middleColumn], 4);
  // setBitOn(maze[middleRow][middleColumn], 3);
  // setBitOff(maze[middleRow][middleColumn], 2);

  showMaze();
  //  if(distance(int pin)()<20){
  //      while(digitalRead(finCourseG) || digitalRead(finCourseD) ){
  //    //Serial.print(digitalRead(finCourseG));
  //    //Serial.println(digitalRead(finCourseD));
  //    forward();
  //  }
  //  stoop();
  //
  // // move(-5,100);
  //  }

  // move(-150,300);
  // forward();
  //       while(digitalRead(finCourseG) || digitalRead(finCourseD) ){
  //     //Serial.print(digitalRead(finCourseG));
  //     //Serial.println(digitalRead(finCourseD));
  //     forward();
  //   }
  //
    //move(300,200);
    //Serial.println("lllllllllllllllllllllllllllllll");
  //
  //
  //  delay(20000);
  // showMaze();

}}
void dropR() {
  servo.attach(45);
  digitalWrite(led, HIGH);
  servo.write(45);
  delay(30);
  servo.write(0);
  delay(10);
  digitalWrite(led, LOW);
}
void dropL() {
  servo.attach(44);
  digitalWrite(led, HIGH);
  servo.write(45);
  delay(30);
  servo.write(0);
  delay(10);
  digitalWrite(led, LOW);
  servo.detach();
}
void performAction(char command)
{
  if (isDigit(command))
  {
    vitesse = (command - '0') * 255 / 9;
  }
  switch (command)
  {
    case 'R':
      right();
      break;
    case 'L':
      left();
      break;
    case 'F':
      forward();
      break;
    case 'B':
      backward();
      break;

    default:
      stoop();
      break;
  }
}
bool button_state = false;

void loop()
{
  //Serial.println("loooooooooooooooooooooooooooooop");
  //go();
  //move(300,200);
  //move(300,200);
  if (Serial.available() > 0) {
    char lettre = Serial.read();
    // Check if the received character is 'Y1'
    if (lettre == 'A') {
      dropR();
    } else if (lettre == 'C') {
      dropL();
    }
    else if ((lettre == 'B') || (lettre == 'D')) {
      go();
    }
  }}
  //orienter(90);
  // move(300, vitesse);
  //go();

  //  Serial.print(dL_total);
  //    Serial.print("      ");
  //   Serial.println(dR_total);
  //  emchi(300);
  //  move(300,255);
  // delay(3000);
  // move(300,255);
  // delay(3000);

  //       while(true){
  //        Serial.print(distance(int pin)());
  //        Serial.print("/    /");
  //        Serial.print(distance(int pin)());
  //        Serial.print("/    / ");
  //        Serial.print(distance(int pin)());
  //        Serial.print("/     /");
  //       Serial.println(distance(int pin)());
  //     }


  //  for (int i = 0; i < 4; ++i)
  //  {
  //    Serial.print(dist[i]);
  //    Serial.print("/     /");
  //  }
  //emchi(300);


  //move(300,500);
  //delay(2000);


  //    Serial.print(dL_total);
  //    Serial.print("      ");
  //   Serial.println(dR_total);
  // delay(2000);
  // if (!button_state && !(digitalRead(finCourseD)))
  //  {
  //    button_state=true;
  //    String ch = check(current_x, current_y);
  //
  //    if (ch == "up")
  //    {
  //      current_x = current_x - 1;
  //    }
  //    else if (ch == "right")
  //    {
  //      current_y = current_y + 1;
  //    }
  //    else if (ch == "down")
  //    {
  //      current_x = current_x + 1;
  //    }
  //    else if (ch == "left")
  //    {
  //      current_y = current_y - 1;
  //    }
  //
  ////    showMaze();
  //  }
  //  if(digitalRead(finCourseD)){
  //    button_state=false;
  //  }
  // if (Serial.available())
  // {
  //   char command = Serial.read();
  //   performAction(command);
  // }

  // if (distance(int pin)() > 15 || distance(int pin)() > 15)
  // {
  //   stoop();
  // }
