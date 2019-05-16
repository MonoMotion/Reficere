/*************************************
  Reficere system for YamaX 7.0 or any robot
  made by Y-modify, Nyanyan & coord.e
  Copyright © 2016-2019 Y-modify All Rights Reserved.
*************************************/

//definition about weight sensors
#define numOfSensor 4
int sensorPort[numOfSensor] = {16, 13, 14, 5};
#define CLK 10 //CLK pin is common
float offset[numOfSensor];
float sensorWeight[numOfServo];
float allWeight;
float centerOfMass[2]; //x,y
#define numOfJoint numOfServo / 2

//numbers and definitions about the robot
#define numOfLink 4
float upperLinkCentroidHeight[numOfLink];//height of centroid of all upper links[mm]
float linkCentroidHeight[numOfLink]; //height of centroid of each link[mm]
float lengh[numOfLink] = {36.0, 49.5, 77.0};//lengh of link[mm]; all must be given
float weight[numOfLink] = {181.01, 0.0, 0.0, 0.0}; //weight of link[g]; first one must be given
float sensorDistance = 110.0 / 2; //unit conversion (sensor value to mm)
float scaleWeight = 0.0;


void StabilizationInit() //setup
{
  for (int i = 0; i < numOfSensor; i++)
    pinMode(sensorPort[i], INPUT);
  pinMode(CLK, OUTPUT);

  getCentroid(20);
  for (int i = 0; i < numOfSensor; i++)
    offset[i] = sensorWeight[i];
}


void setScaleWeight() //setting scale weight is essential to measure exact Centroid
{
  delay(5000);
  getCentroid(20);
  scaleWeight = allWeight;
  Serial.println(scaleWeight);
  Serial.println("scaleWeight complete");
}



void Reficere(int link) //main function
{
  //reset
  delay(5000);
  convert(reset, 40, 10, 1);
  delay(1000);

  //definition of numbers
  float deg[3] = {0.0, 90.0, 0.0}; //alpha_n, alpha_n-1, beta
  double upperWeightSum = 0;
  for (int j = 0; j < link; j++)
    upperWeightSum += weight[j];

  //1 set upper links' height of centroid
  if (upperLinkCentroidHeight[link - 1] == 0)
    upperLinkCentroidHeight[link - 1] = measurementA(link - 1, 10);
  delay(1000);
  //upperLinkCentroidHeight[link - 1] = 18.2;

  //2 set alpha_n & alpha_n-1
  deg[0] = measurementB(link, deg[1]);
  deg[1] = deg[1] - deg[0];
  delay(1000);

  //3 set beta
  deg[2] = measurementC(link, deg[0], deg[1], upperWeightSum);

  //get robot's all weight
  getCentroid(15);

  //calculate centroid height of this link
  linkCentroidHeight[link] = lengh[link] + (upperLinkCentroidHeight[link - 1] / tan((deg[2] + deg[1]) * PI / 180)); //for URDF

  //calculate weight of this link
  weight[link] = (upperWeightSum * (upperLinkCentroidHeight[link - 1] * cos(deg[0] * PI / 180) - lengh[link] * sin(deg[0] * PI / 180)) * tan((deg[2] + deg[1]) * PI / 180))
                 / ((lengh[link] * tan((deg[2] + deg[1]) * PI / 180) + upperLinkCentroidHeight[link - 1]) * sin(deg[0] * PI / 180));

  //calculate upper links' centroid height
  upperLinkCentroidHeight[link] = linkCentroidHeight[link] + (lengh[link] + upperLinkCentroidHeight[link - 1] - linkCentroidHeight[link - 1]) * upperWeightSum / (upperWeightSum + weight[link]);

  //output
  for (int i = 0; i < sizeof(deg) / sizeof(float); i++)
  {
    Serial.print(deg[i], 10);
    Serial.print("\t");
  }
  Serial.print(upperLinkCentroidHeight[link - 1], 10);
  Serial.print("\t");
  Serial.print(linkCentroidHeight[link], 10);
  Serial.print("\t");
  Serial.print(lengh[link], 10);
  Serial.print("\t");
  Serial.print(allWeight, 10);
  Serial.print("\t");
  Serial.print(upperWeightSum, 10);
  Serial.print("\t");
  Serial.print(linkCentroidHeight[link], 10);
  Serial.print("\t");
  Serial.println(weight[link], 10);

  //reset
  convert(reset, 30, 30, 1);
}





float measurementA(int link, int unit) //1 set upper links' height of centroid
{
  //reset
  convert(reset, 30, 30, 1);
  delay(1000);

  //set default centroid
  getCentroid(20); //get default
  float Gdefault[2] = {centerOfMass[0], centerOfMass[1]}; //xl,yl/xr,yr

  //numbers
  float degmin = 90; //max degree to move
  float degmax = -90; //min degree to move
  float cut = 1; //how much to move one time
  int times = abs(degmax - degmin) / cut;
  int excludeDeg = 30; //if moving degree is too small, the deta will not be the exact
  int exclude = 2 + unit / 10;
  int moveServo[2] = {linkToJoint(link, 0), linkToJoint(link, 1)}; //servo number
  boolean state = 0;
  int count = 0;
  float hSum[unit + (exclude * 2)]; //data
  int sumSize = sizeof(hSum) / sizeof(float); //use average excluding highest and lowest 10%


  for (int i = 1; i <= abs(degmin); i++)// first, set the position.
  {
    smoothmotion(moveServo[0], degmin ,  abs(degmin), i, 1);
    smoothmotion(moveServo[1], degmin ,  abs(degmin), i, 1);
    delay(20);
  }
  delay(300);

  while (count < sumSize)// main
  {
    for (int i = 1; i <= times; i++)
    {
      if (count < sumSize)
      {

        if (state == 0)
        {
          smoothmotion(moveServo[0], degmax , times, i, 0);
          smoothmotion(moveServo[1], degmax , times, i, 0);
        }
        if (state == 1)
        {
          smoothmotion(moveServo[0], degmin , times, i, 0);
          smoothmotion(moveServo[1], degmin , times, i, 0);
        }

        float nowdeg = formerDeg[moveServo[0]] * degmax / abs(degmax);

        if (nowdeg <= degmin || nowdeg >= degmax)
          state = !state;

        delay(50);
        if (nowdeg >= excludeDeg || nowdeg <= -excludeDeg)
        {
          delay(450);
          getCentroid(15);

          //calculate
          float upperWeightSum = 0;
          for (int j = 0; j <= link; j++)
            upperWeightSum += weight[j];
          hSum[count] = allWeight * (centerOfMass[1] - Gdefault[1]) * sensorDistance / (upperWeightSum * sin(nowdeg * PI / 180));

          //output
          Serial.print(i); //print times
          Serial.print("\t");
          Serial.print((float)nowdeg); //print deg
          Serial.print("\t");
          Serial.print(hSum[count], 10); //is this same in all time?
          Serial.println("");

          count++;
        }
      }
    }
  }

  // calculate average
  sort(hSum, sizeof(hSum) / sizeof(float));
  float sum = 0;
  for (int k = 0; k < unit; k++)
    sum = sum + hSum[exclude + k];
  sum = sum / unit;

  //output
  Serial.println(sum, 10);
  Serial.println("experiment1 finished");

  //reset
  convert(reset, 30, 30, 1);
  return sum;
}






float measurementB(int link, int theta)//2 set alpha_n & alpha_n-1
{
  convert(reset, 30, 30, 1);
  delay(1000);
  getCentroid(20); //get default
  float Gdefault[2] = {centerOfMass[0], centerOfMass[1]}; //xl,yl/xr,yr

  float procDeg = 0;
  float P = 0;
  float I = 0;
  float D = 0;
  float pastP = 0;
  float Kp = 4;
  float volume = 4;
  float threshold = 0.01;
  float thresholdD = 0.01;

  int times = abs(theta);
  for (int j = 1; j <= times; j++) //thetaN-1適用
  {
    smoothmotion(linkToJoint(link - 1, 0), theta, times, j, 1);
    smoothmotion(linkToJoint(link - 1, 1), theta, times, j, 1);
    delay(10);
  }
  delay(1000);
  for (;;)
  {
    delay(100);
    getCentroid(15);
    if (-threshold < centerOfMass[1] - Gdefault[1] && centerOfMass[1] - Gdefault[1] < threshold && -thresholdD < D && D < thresholdD)
      break;

    P = centerOfMass[1] - Gdefault[1];
    D = P - pastP;
    pastP = P;
    procDeg = Kp * P * volume;
    Serial.print(P, 10);
    Serial.print("\t");
    Serial.print(formerDeg[linkToJoint(link, 1)], 10);
    Serial.println("");
    setServoPulse(linkToJoint(link, 0), formerDeg[linkToJoint(link, 0)] + procDeg);
    setServoPulse(linkToJoint(link, 1), formerDeg[linkToJoint(link, 1)] + procDeg);
  }
  Serial.println("experiment2 finished");
  return -formerDeg[linkToJoint(link, 0)];
}







float measurementC(int link, float degA, float degB, float upperWeightSum)//betaを求める
{
  convert(reset, 30, 30, 1);
  delay(1000);
  getCentroid(20); //get default
  float Gdefault[2] = {centerOfMass[0], centerOfMass[1]}; //xl,yl/xr,yr
  double deg[4] = {degA, degB, 60.0, 0.0}; //deg[2]is beta, deg[3] is gamma

  float difference = 200;
  float procDeg = 0;
  float P = 0;
  float D = 0;
  float pastP = 0;
  float Kp = 4;
  float Kd = 0;
  float volume = 0.003;
  float threshold = 3.0;
  float thresholdD = 10.0;

  deg[3] = asin(-cos(deg[2] * PI / 180) / tan((deg[2] + deg[1]) * PI / 180)) * 180 / PI;

  int times = 90;
  for (int j = 1; j <= times; j++) //thetaN-1適用
  {
    smoothmotion(linkToJoint(link - 1, 0), (90 - deg[2]) + deg[3], times, j, 1);
    smoothmotion(linkToJoint(link - 1, 1), (90 - deg[2]) + deg[3], times, j, 1);
    smoothmotion(linkToJoint(link, 0), -(90 - deg[2]), times, j, 1);
    smoothmotion(linkToJoint(link, 1), -(90 - deg[2]), times, j, 1);
    delay(10);
  }
  delay(1000);

  for (;;)
  {
    if (-threshold < difference && difference < threshold && -thresholdD < D && D < thresholdD)
      break;
      
    delay(500);
    getCentroid(15); //get default
    float distance = (centerOfMass[1] - Gdefault[1]) * sensorDistance;
    float f = allWeight * distance * tan((deg[2] + deg[1]) * PI / 180) * sin(deg[0] * PI / 180);
    float g = upperWeightSum * upperLinkCentroidHeight[link - 1] * cos(deg[2] * PI / 180) * (cos(deg[0] * PI / 180) * tan((deg[2] + deg[1]) * PI / 180) + sin(deg[0] * PI / 180));
    difference = f - g;

    P = difference;
    D = P - pastP;
    pastP = P;
    float procDeg = Kp * P * volume;
    //float procDeg = 1;
    deg[2] += procDeg;
    double tmp = -cos(deg[2] * PI / 180) / tan((deg[2] + deg[1]) * PI / 180);
    if (abs(tmp) > 0.9)
      break;
    deg[3] = asin(tmp) * 180 / PI; //度に変換

    Serial.print(deg[2], 10);
    Serial.print("\t");
    Serial.print(deg[3], 10);
    Serial.print("\t");
    Serial.print(distance, 10);
    Serial.print("\t");
    Serial.print(f, 10);
    Serial.print("\t");
    Serial.print(g, 10);
    Serial.print("\t");
    Serial.print(difference, 10);
    Serial.println("");

    setServoPulse(linkToJoint(link - 1, 0), (90 - deg[2]) + deg[3]);
    setServoPulse(linkToJoint(link - 1, 1), (90 - deg[2]) + deg[3]);
    setServoPulse(linkToJoint(link, 0), -(90 - deg[2]));
    setServoPulse(linkToJoint(link, 1), -(90 - deg[2]));
  }

  Serial.println("experiment3 finished");
  return deg[2];//beta出力
}


int linkToJoint(int link, int LR) //そのリンクの下についているモーターの番号を返す。0=L, 1=R
{
  if (LR == 0)
    return link + 3;
  if (LR == 1)
    return link;
}



void sort(float data[], int datasize) //sort
{
  for (int i = 0; i < datasize; i++)
  {
    for (int j = i + 1; j < datasize; j++)
    {
      if (data[i] > data[j])
      {
        float tmp =  data[i];
        data[i] = data[j];
        data[j] = tmp;
      }
    }
  }
}

void getCentroid(int averageTimes) //get each sensor's weight and calculate the centroid
{
  int exclude = 1 + (averageTimes / 10);
  float upperWeightSum[numOfSensor][averageTimes + exclude * 2];

  for (int i = 0; i < averageTimes + exclude * 2; i++) //get weight
  {
    for (int j = 0; j < numOfSensor; j++)//sensorNum
      upperWeightSum[j][i] = (float)Read(j);
  }

  for (int i = 0; i < numOfSensor; i++)//sort
  {
    for (int j = 0; j < averageTimes + exclude * 2; j++)
    {
      for (int k = j + 1; k < averageTimes + exclude * 2; k++)
      {
        if (upperWeightSum[i][j] > upperWeightSum[i][k])
        {
          float tmp =  upperWeightSum[i][j];
          upperWeightSum[i][j] = upperWeightSum[i][k];
          upperWeightSum[i][k] = tmp;
        }
      }
    }
  }

  for (int i = 0; i < numOfSensor; i++)//sum
  {
    sensorWeight[i] = 0;
    for (int j = 0; j < averageTimes; j++)//sensorNum
      sensorWeight[i] = sensorWeight[i] + upperWeightSum[i][exclude + j];
  }

  for (int i = 0; i < numOfSensor; i++)//average
    sensorWeight[i] = sensorWeight[i] / averageTimes;

  allWeight = 0;
  for (int i = 0; i < numOfSensor; i++)
    allWeight = allWeight + sensorWeight[i];
  allWeight = allWeight - scaleWeight;

  float x = -1 + 2 * (sensorWeight[1] + sensorWeight[3]) / (sensorWeight[0] + sensorWeight[1] + sensorWeight[2] + sensorWeight[3]); //-1~1 x
  float y = -1 + 2 * (sensorWeight[0] + sensorWeight[1]) / (sensorWeight[0] + sensorWeight[1] + sensorWeight[2] + sensorWeight[3]); //-1~1 y//見かけ上の重心
  centerOfMass[0] = scaleWeight * x / allWeight + x;
  centerOfMass[1] = scaleWeight * y / allWeight + y;//はかり自体の重さを補正
}


float Read(int sensorNum) //read the weight of a sensor
{
  long data = 0;
  while (digitalRead(sensorPort[sensorNum]) != 0);
  for (char i = 0; i < 24; i++)
  {
    digitalWrite(CLK, 1);
    delayMicroseconds(1);
    digitalWrite(CLK, 0);
    delayMicroseconds(1);
    data = (data << 1) | (digitalRead(sensorPort[sensorNum]));
  }
  digitalWrite(CLK, 1); //gain=128
  delayMicroseconds(1);
  digitalWrite(CLK, 0);
  delayMicroseconds(1);
  data = data ^ 0x800000;
  float volt; float gram;
  volt = data * (4.2987 / 16777216.0 / 128); //Serial.println(volt, 10);
  gram = volt / ((0.000669 * 4.2987) / (400.0 * 1.0909)); //Serial.println(gram, 4);
  return gram - offset[sensorNum];
}
