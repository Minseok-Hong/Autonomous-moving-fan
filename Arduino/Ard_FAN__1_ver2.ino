/*
    四生FAN



*/


#include <NoBlind_ultrasonic.h>
#include <SHT1x.h>
#include <string.h>
#include <Math.h>
#include <Servo.h> 

//PIN defines
#define SONIC_SERVO 3
#define LEFT_SERVO  5
#define RIGHT_SERVO 6
#define TRIGGER_PIN   8
#define ECHO_PIN      9
#define DataPin 10
#define ClockPin 11
//Various time delays used for driving and servo
#define maxForwardSpeed 120 // Max speed we want moving forward
#define maxBackwardSpeed -120 // Max reverse speed 
#define Stop 90 // Stop Value
#define ECHO_DISTANCE  25  //maximum distance
#define MAXPATH 500 //maximum distance
#define moveDelay 930
#define FoDelay 1365

#define stateWAITING    0
#define stateMOVING     1
#define stateSTOP       2
#define stateARRIVE     3

#define errorDETECT     1


//Ultrasonic & Temperature/Humid Sensor
NoBlind_Ultrasonic sonar(TRIGGER_PIN, ECHO_PIN, ECHO_DISTANCE);
SHT1x sht1x(DataPin, ClockPin);

//Struct for Node
typedef struct Node{
  int f = 0;    // full distance from start to goal
  int g = 0;    // distance from start to node
  char state = 0; // none, open, close (0, 1, 2)state
  int camefrom = 0; // parents node
}Node;


bool forward();
void backward();
void right();
void left(); 
void halt();
void TempHumid();
void operationOfFan();
void moving(Node* node, int start, int arrayWidth, int maxCost, int* obstacle);
int Astar(Node* node, int start, int arrayWidth, int maxCost);
void obstacleDetect(Node* node, int current, int arrayWidth, int* obstacle, int maxCost, int pastMode, int pathIndex);
int h(int start, int t_goal, int arrayWidth);
int build_Path(Node* node, int start, int* camefrom, int currentNode, int fScore, int maxCost);

/*
 * Global Variable
 */
//servo
Servo motor_left; // Create Left servo motor object
Servo motor_right; // Create Right servo motor object
Servo motor_sonic; // Create ultrasonic servo motor object

//array for A*
// the reason for using one-dimesional array is the length and breadth is changeable
//  if we delare for two -dimensional array , it leads to memory space waste.

int path[MAXPATH]; //Path
int openSet[MAXPATH]; // openSet list
int camefrom[MAXPATH];  // parent list


int goal;   // goal
int current;  // moving path current Index
int closeSetNum = 0;    //closeSet(stack) pop Number
int moveIndex = 0;      // 

//room Information
//arrayWidth & maxCost(width*height)
/*
int start;
int arrayWidth;
int maxCost;
*/

/*
 * Setup
 *  .Motor attach
 *  .Pin Setting
 */
void setup() 
{
    // Attach the servo to the digital pin
    motor_left.attach (LEFT_SERVO);     // continuous Servo
    motor_right.attach(RIGHT_SERVO);    // continuous Servo
    motor_sonic.attach(SONIC_SERVO);    // ultrasonic servo pin (sg-90)
    // Ultrasonic Pin
    pinMode(TRIGGER_PIN,OUTPUT); // Trigger is an output pin 
    pinMode(ECHO_PIN,INPUT); // Echo is an input pin 
    pinMode(2, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(4, OUTPUT);
    motor_sonic.write(83);  
    Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps

    halt();
}

/*
 * Loop
 *  .Receive input
 *  .Send temperature, humidity, Remaining distance, State, Unexpected occurrence
 *  
 *  .Variable
 *      
 *  
 */
void loop() 
{
    int start;
    int arrayWidth;
    int maxCost;
    int inputMoving;   
    int index;
    Node node[MAXPATH];   // Node list
    int obstacle[MAXPATH];

    // Serial communication
    if(Serial.available()) 
    {
        char rd;
        rd = Serial.read();

        // receive data 
        if(rd == 'I')   // Send Temperature and Humidity
        {
            delay(200);
            TempHumid();        // Temperature and Humidity
            Serial.print(stateWAITING);  // State ( Moving )
            Serial.print(",");
            Serial.print("0"); // Remain distance
            Serial.print(",");
            Serial.print("0");  // Total distance
            Serial.print(",");
            Serial.println("0");  // No path (error)
        }
        if(rd == 'M')   // Receive input for driving
        {
            //Serial.println("O");
            index = 0;
            delay(2000);
            if(Serial.available())
            {
                start = goal = arrayWidth = maxCost = 0;
                memset(obstacle, 0, sizeof( int ) * MAXPATH ); 
                inputMoving = 0;
                inputMoving = Serial.parseInt();
                while(  inputMoving )
                {
                    switch(index)
                    {
                        case 0:     // receive Start location
                            start = inputMoving;
                            index++;
                            break;
                        case 1:     // receive Destination (Goal)
                            goal = inputMoving;
                            index++;
                            break;
                        case 2:      // receive Width for array
                            arrayWidth = inputMoving;
                            index++;
                            break;
                        case 3:     // receive Max Cost
                            maxCost = inputMoving;
                            index++;
                            break;
                        default:    // receive obstacle location
                            //1 158 10 222 2 3 4 5 6 7 8
                            obstacle[closeSetNum] = inputMoving;
                            node[obstacle[closeSetNum]].state = 2;  // set CloseSet
                            closeSetNum++;
                            break;
                    }
                    inputMoving = Serial.parseInt();
                }                
            }
            TempHumid();
            Serial.print(stateMOVING);  // State ( Moving )
            Serial.print(",");
            Serial.print("0"); // Remain distance
            Serial.print(",");
            Serial.print("0");  // Total distance
            Serial.print(",");
            Serial.println("0");  // No path (error)
            operationOfFan();       // operate the Fan
            moving(node, start, arrayWidth, maxCost, obstacle);
            halt();
        }      
    }


}


/*
 * TempHumid
 * 
 * Print Temperature and Humidity
 * 
 */
void TempHumid() 
{
    int TempC = 0;
    int Humid = 0;
  
    TempC = sht1x.readTemperatureC() - 5;
    Humid = sht1x.readHumidity();

    Serial.print(TempC);
    Serial.print(",");
    Serial.print(Humid);
    Serial.print(",");
}

/*
 * moving
 * 
 * Calling the Astar function, calculate the shortest path 
 * moves across the path =
 * 
 * if it detects the obstacle, it sets the current location as the start point , and recalls the function.
 * 
 */
void moving(Node* node, int start, int arrayWidth, int maxCost, int* obstacle)
{
    char pastMode, currentMode; // modes according to movement 
    int caculateTmpX, caculateTmpY; // temporary storage variables to calculate the path.
    int pathIndex = 0;  // 
    int hScore = 0;


    current = moveIndex;    // current location value from path arrays
    pathIndex = Astar(node, start, arrayWidth, maxCost);   

    if(pathIndex <= 0)
    {
        TempHumid();            // Temperature and Humidity
        Serial.print(stateMOVING);  // State ( Moving )
        Serial.print(",");
        Serial.print(hScore); // Remain distance
        Serial.print(",");
        Serial.print(pathIndex);  // Total distance
        Serial.print(",");
        Serial.println(errorDETECT);  // No path (error)
    }
    
    while(path[current] != goal && current < pathIndex + 1) //  Until Arrival 
    {
        hScore = node[path[current]].f - node[path[current]].g;
        TempHumid();            // Temperature and Humidity
        Serial.print(stateMOVING);  // State ( Moving )
        Serial.print(",");
        Serial.print(path[current]); // Remain distance
        Serial.print(",");
        Serial.print(pathIndex);  // Total distance
        Serial.print(",");
        Serial.println(errorDETECT);  // No path (error)
        delay(200);

        /*
         *  forward / backward / right / left check
         */
        // check the current location from the tracked path  
        caculateTmpX = (path[current]/arrayWidth) - (path[current - 1]/arrayWidth);
        caculateTmpY = (path[current]%arrayWidth) - (path[current - 1]%arrayWidth);
        pastMode = abs(caculateTmpX)*(1-caculateTmpX) + abs(caculateTmpY)*(2 + caculateTmpY);
        // check how the fan will track path 
        caculateTmpX = (path[current + 1]/arrayWidth) - (path[current]/arrayWidth);
        caculateTmpY = (path[current + 1]%arrayWidth) - (path[current]%arrayWidth);
        currentMode = abs(caculateTmpX)*(1-caculateTmpX) + abs(caculateTmpY)*(2 + caculateTmpY);

        if(current != 1) //  if it is not the first point, it calculates it according to logic.
            currentMode = (3*pastMode + currentMode) % 4;     

        // move forward /backward or rotation based on the calculated direction
        switch(currentMode){
            case 0:
                if(!forward()) // forward
                {
                    obstacleDetect(node, current, arrayWidth, obstacle, maxCost, pastMode, pathIndex);                    
                    return;
                }
                //delay(moveDelay);
                halt();
                break;            
            case 1:
                right();        // right
                halt();
                if(!forward()) 
                {
                    obstacleDetect(node, current, arrayWidth, obstacle, maxCost, pastMode, pathIndex);    
                    return;
                }
                //delay(moveDelay);
                halt();
                break;
            case 2:
                backward();     // backward
                halt();
                if(!forward()) 
                {
                    obstacleDetect(node, current, arrayWidth, obstacle, maxCost, pastMode, pathIndex);    
                    return;
                }
                //delay(moveDelay);
                halt();
                break;
            case 3:
                left();         // left
                halt();
                if(!forward()) 
                {
                    obstacleDetect(node, current, arrayWidth, obstacle, maxCost, pastMode, pathIndex);    
                    return;
                }
                //delay(moveDelay);
                halt();
                break;
            default:
                break;
        }

        // check if the value is goal
         if(path[current + 1] == goal)
            break;
        current++;
    }
}

/*
 * ## obstacleDetect ##
 * 
 * Initialize Variable
 *   .fScore and gScore for each Nodes
 *   .closeSet and openSet
 *   .up to current Node
 * Mark the node FAN is looking at as an obstacle
 * moving function starting from current
 */
void obstacleDetect(Node* node, int current, int arrayWidth, int* obstacle, int maxCost, int pastMode, int pathIndex)
{
    int detectObstacle = 0;
    int pastMoveX, pastMoveY;
    long Duration = 0;

    halt();

    // calculate the direction FAN is looking at
    switch(pathIndex)
    {
        case 0:
            pastMoveX = 1;
            break;
        case 1:
            pastMoveX = -1;
            break;
        case 2:
            pastMoveY = -1;
            break;
        case 3:
            pastMoveY = 1;
            break;        
    }
    detectObstacle = (((path[current]/arrayWidth) + (pastMoveX*2))*arrayWidth)
    + ((path[current] % arrayWidth) + (pastMoveY*2));
    obstacle[closeSetNum] = detectObstacle;  // Mark the node FAN is looking at as an obstacle
    closeSetNum++;

    // Initialize
    for(int k = current; k < pathIndex; k++)
        path[k] = 0;
    moveIndex = current;
    memset(node, 0, sizeof(struct Node) * MAXPATH);   
    for(int k = 0; k < closeSetNum; k++)
        node[obstacle[k]].state = 2;

    delay(moveDelay);
    moving(node, path[current-1], arrayWidth, maxCost, obstacle);
}

/*
 * Distance
 * 
 * Calculates the Distance in mm using ultrasonic Sensor
 */
long Distance(long time) 
{ 
  // Calculates the Distance in mm 
  // ((time)*(Speed of sound))/ toward and backward of object) * 10 
  long DistanceCalc; // Calculation variable 
  DistanceCalc = ((time /2.9) / 2); // Actual calculation in mm 

  return DistanceCalc; // return calculated value 
} 


/*
 * ## A star algorithm ##
 * 
 * Reference
 *   .Pseudocode in https://en.wikipedia.org/wiki/A*_search_algorithm
 *   
 * f g h
 */
int Astar(Node* node, int start, int arrayWidth, int maxCost)
{
    int currentNode = start; // currentNode initialize start
    int maxCameIndex = 0;
    int n_fScore = 0;
    int n_gScore = 0; // neighbor fScore and gScore
    int openSetNum = 0;
    int neighbor;
    int neighborIndex = 0;    // neighbor

    memset(openSet, 0,sizeof( int ) * MAXPATH );
    memset(camefrom, 0,sizeof( int ) * MAXPATH );

    node[start].state = 1;
    openSet[openSetNum] = start;
    openSetNum++;

    // Start Node f and g Score
    node[start].f = h(start, goal, arrayWidth); // Start f Score = h Score
    node[start].g = 0;

    
    while(openSetNum >= 0)
    {
        currentNode = openSet[0];
        if(currentNode == goal) // End Moving
            return build_Path(node, start, camefrom, currentNode, node[goal].f, maxCost);

        // openSet`s 0 index is out, and Rearrange
        openSet[0] = 0;
        for(int index = 0; index < openSetNum; index++)
        {
            if( index == openSetNum - 1)
            {
                openSet[index] = 0;
                break;
            }
            else
            {
                openSet[index] = openSet[index+1];
            }
        }
        openSetNum--;
        node[currentNode].state = 2;  // mark obstacle

        // neighbor Node search
        for(neighborIndex = 0; neighborIndex < 4; neighborIndex++)
        {
            int arrayX = currentNode / arrayWidth;
            int arrayY = currentNode % arrayWidth;
            
            switch(neighborIndex)
            {
            //Neighbor Checking
            case 0:
                if((arrayY - 1) > 0)
                {
                    neighbor = currentNode - 1;
                    break;
                }
                else
                    continue;
            case 1:
                if((arrayY + 1) < arrayWidth)
                {
                    neighbor = currentNode + 1;
                    break;
                }
                else
                    continue;
            case 2:
                if((arrayX - arrayWidth) > 0){
                    neighbor = currentNode - arrayWidth;
                    break;
                }
                else
                    continue;
            case 3:
                if((arrayX +  arrayWidth) < maxCost) // 
                {
                    neighbor = currentNode + arrayWidth;
                    break;                        
                }
                else
                    continue;      
            default:
                break;
            }
            // if neighbor state is close, continue
            if(node[neighbor].state == 2)
                continue;

            // Calculate fScore and gScore for neighbor node
            n_gScore = node[currentNode].g + h(currentNode, neighbor, arrayWidth);
            n_fScore = n_gScore + h(neighbor, goal, arrayWidth);

            // neighbor is none , state is 0
            if(node[neighbor].state == 0)
            {
                // Insertion Sort for neighbor node in openSet
                for(int index = openSetNum; index >= 0; index--)
                {
                    if(openSet[index] == 0 && index == 0)
                        openSet[index] = neighbor;
                    else
                    {
                        int indexJ = index - 1;
                        if(indexJ == 0)
                        {
                            if(node[openSet[0]].f > n_fScore)
                            {
                                openSet[1] = openSet[0];
                                openSet[0] = neighbor;
                                break;
                            }
                            else
                            {
                                if(node[openSet[0]].f == n_fScore)
                                    openSet[1] = openSet[0];
                                else
                                {
                                    openSet[1] = neighbor;
                                    break;                      
                                }
                            }
                        }
                        else
                        {
                            if(node[openSet[indexJ]].f > n_fScore)
                                openSet[indexJ + 1] = openSet[indexJ];
                            else
                            {
                                if(node[openSet[indexJ]].f == n_fScore)
                                    openSet[indexJ+1] = openSet[indexJ];
                                else
                                {
                                    openSet[indexJ+1] = neighbor;
                                    break;
                                }
                            }
                        }
                    }
                }
                openSetNum++;
                node[neighbor].state = 1;  // neighbor`s state change state to openNode
            }
            else if(n_gScore >= node[neighbor].g)
                continue;

            // assign value to neighbor node
            node[neighbor].g = n_gScore;
            node[neighbor].f = n_fScore;
            node[neighbor].camefrom = currentNode;
        }            
    }
    //free(openSet);
    //free(camefrom);
    
    return 0; //fail    
}

/*
 * ## h Score Calculate Function ##
 */
int h(int start, int t_goal, int arrayWidth)
{
    int x_dis;
    int y_dis;
    
    x_dis = abs((start/arrayWidth) - (t_goal/arrayWidth));
    y_dis = abs((start%arrayWidth) - (t_goal%arrayWidth));
    
    return (x_dis + y_dis);
}

 /*
  * buildPath
  * 
  * Find the way back from goal node to start node
  */  
int build_Path(Node* node, int start, int* camefrom, int currentNode, int fScore, int maxCost)
{
    int BuildCurrentNode;
    int pathIndex;
    int index = 0;    int temp;

    if (fScore < 1)
        return 0;   // No path

    BuildCurrentNode = currentNode;
    path[fScore - index + moveIndex] = goal;
    while(BuildCurrentNode != start)
    {
        path[fScore - index + moveIndex - 1] = node[BuildCurrentNode].camefrom;                     
        BuildCurrentNode = node[BuildCurrentNode].camefrom;
        index++;
        temp++;
    }

    path[0] = start;


    Serial.print("Path: ");
    while(temp>0){
        Serial.print(path[temp + moveIndex+ 1]);
        Serial.print(", ");
        temp--;
    }
    pathIndex = index + moveIndex;
    return pathIndex;
}

/*
 * operationOfFan
 * 
 * operate the Fan
 */
void operationOfFan()
{
    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(12, HIGH);
}

void stopFan()
{
     digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(12, LOW);
}

/*
 * move Function
 * 
 * forward
 *   .right : BackwardSpeed
 *   .left  : ForwardSpeed
 *  if moteor is opposite, so the velocity is
 */
bool forward()
{
    long Duration = 0;
    int index = 0;
    int forwardDelay = moveDelay / 4;

    for(index = 0; index < 4; index++)
    {
        digitalWrite(TRIGGER_PIN, HIGH); // Trigger pin to HIGH 
        delayMicroseconds(10); // 10us high 
        digitalWrite(TRIGGER_PIN, LOW); // Trigger pin to HIGH 
        
        Duration = pulseIn(ECHO_PIN,HIGH); // Waits for the echo pin to get high 
        // returns the Duration in microseconds 
        
        long Distance_mm = Distance(Duration); // Use function to calculate the distance 
        if(Distance_mm < 250)
        {
            halt();
            // Stop and One more Check Obstacle
            digitalWrite(TRIGGER_PIN, HIGH); // Trigger pin to HIGH 
            delayMicroseconds(10); // 10us high 
            digitalWrite(TRIGGER_PIN, LOW); // Trigger pin to HIGH 
            
            Duration = pulseIn(ECHO_PIN,HIGH); // Waits for the echo pin to get high 
            // returns the Duration in microseconds 
            long Distance_mm = Distance(Duration); // Use function to calculate the distance 
            if(Distance_mm < 250)
                break;
        }
        motor_right.write(maxBackwardSpeed);
        motor_left.write(maxForwardSpeed);  
        delay(forwardDelay);
    }
   // halt();
    
    // FAN detect obstacle
    // backward as far as FAN forward
    if(index < 4)
    {   
        int backwardDelay = forwardDelay * (index + 1);
        motor_left.write (maxBackwardSpeed);
        motor_right.write(maxForwardSpeed);
        delay(backwardDelay);
       // halt();
        return false;
    }
    return true;    
}

void backward()
{
    motor_left.write (maxBackwardSpeed);
    motor_right.write(maxForwardSpeed);
    delay(moveDelay);
}

void left()
{
    motor_right.write(maxBackwardSpeed);
    motor_left.write (maxBackwardSpeed);
    delay(moveDelay + 200);
}

void right()
{
    motor_right.write(maxForwardSpeed);
    motor_left.write (maxForwardSpeed);
    delay(moveDelay-30);
}

void halt()
{
    motor_right.write(Stop);
    motor_left.write(Stop);
}
