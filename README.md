# Autonomous-moving-fan

1. Summary

Our Final Project is ‘ An automatic moving fan ‘ which is devised for the man in the one-room. In the summer(Especially this summer so scorching), most of the people is tired of everything and hates to move around, just Rest. In this Project, we just give some input(the size of Room, the relocation of the obstacles) to the Android App.. The fan identified the preset obstacles ( furniture, desk, huge object, etc ….) and avoid them, and then it moves across the path to the user destination.

<div>
  <img width = "700" src = "https://user-images.githubusercontent.com/31812857/44588913-fe49b380-a7f1-11e8-8563-925ef862aea9.png">
 </div>

2. Scenario
  1. In the App, We input the size of Room ( length: 00 breadth: 00)
  2. In the App , The data is transmitted to the Android App.
  3. Touch LCD, we inputs the use time through KeyPad and output results on the TextLCD.
  4. go to the next screen and touch “start” to start moving the fan.
  5. move across the path and finally reach to the user destination
  
  3 Algorithm
Algorithm: the main operation of our fan and instruction set of encountering the obstacles and
processing to the inputs from Android App or Touch LCD.
In constructing algorithm, we research many algorithm and ask for some advices from excellent
teaching assistants. Our assistant gives solution “ BFS algorithm “ and “ A* (A star) algorithm.
We devise the algorithm based on BFS algorithm but we find many restrictions. So we give up
first algorithm and re-devised the algorithm based on A* (A star) algorithm.
In A* algorithm, it wisely search the neighborly nodes and calculates some weight and
compares from them . This above function is similar to BFS algorithm but the great point is that
A* algorithm repeatedly considers the shortest path from the destination, but BFS algorithm
doesn’t do that.
A* is an informed search algorithm, or a best-first search, meaning that it solves problems by
searching among all possible paths to the solution (goal) for the one that incurs the smallest cost
(least distance travelled, shortest time, etc.), and among these paths it first considers the ones
that appear to lead most quickly to the solution. It is formulated in terms of weighted graphs:
starting from a specific node of a graph, it constructs a tree of paths starting from that node,
expanding paths one step at a time, until one of its paths ends at the predetermined goal node.
At each iteration of its main loop, A* needs to determine which of its partial paths to expand into
one or more longer paths. It does so based on an estimate of the cost (total weight) still to go to
the goal node. Specifically, A* selects the path that minimizes

f(n) = g(n) + h(n)

where n is the last node on the path, g( n) is the cost of the path from the start node to n, and
h( n) is a heuristic that estimates the cost of the cheapest path from n to the goal. The heuristic is
problem-specific. For the algorithm to find the actual shortest path, the heuristic function must be
admissible, meaning that it never overestimates the actual cost to get to the nearest goal node.
Output Screen:>
A* algorithm

4. Embedded
Our function of each component in Embedded board is below.
1. Keypad
We use the rdata value to get the value of the matrix 4*4.(we only need 10 because we need
0 ~ 9). Touch saves the three digits in turn to the Number array and receives the values as
result in three digits.
2. Buzzer
If we click Start Button, it starts with melody
3. 7 Segment
7 segment can show current date. In Main( ) function, if we implement 7 segment ,it declares
t_date (int data type variable), cmd_buffer ( char[] datatype variable) to pass instruction by
system function. And in t_date, it receives from Android App, and convert year, month,
day(global variable into int data type as atoi ( ) function) and also convert into numbers to
display in screen. And then using sprint (), we put t_date value to %d(int data type) ./fndtest
s 1% d string and store on cmd_buffer, and after calling in system(cmd_buffer), it displays on
7 segment screen.


4. Dot Matrix
The Dot matrix outputs the usage time. In the main () function, use the thread to execute
the dot function. it will be displayed on the Dot Matrix by increasing the usage time every
minute.
5. Text LCD
Text LCD receives temperature and humidity information from Arduino and outputs it. In the
main () function, use the thread to execute the text_lcd function. If the current state is not
waiting, output the temper and humid values declared as global variables to the Text LCD
using the system function.
6. OLED
Comparing previous user state, if change , print out on OLED Screen img file responding to
state .
7. BUS LED
Bus Led outputs how far to the destination when arduino is moving. After getting the total
distance and the remaining distance from Arduino, calculate the ratio between the distance
and assign it to remainPerTotal. And it turns the LED on or off according to the value of the
remainPerTotal variable.
8. Dip/Push SW
Dip/Push SW functions to forcibly terminate the Arduino. In the main () function, use thread
to execute the dip_sw function. while continuing to check the state of the Dip/Push SW, if
it makes clear that just first left switch is on, it commands stop to adurino, and change the
state of FINISHING.
9. Full Color LED
Comparing previous user state, if change , print out on Full Color LED color responding to
state .
10. Camera
In the camera.c code, the CreateCamera function is a function that functions as a create
camera, and the DestroyCamera () function is a function that functions as an end camera.
The startPreview function is a function that functions as a setting initial screen of camera.
The DrawFromRGB656 function is a function that functions as a function of extract rgb
values of a current screen taken by the camera. The cameraview function is the main
function showing real time video taken by the camera. Fileheader_init (unsigned char *
bmp_fileheader, DWORD bfsize) is a function that functions as an initiating making setting
file. Infoheader_init (BITMAPINFOHEADER * bmp_infoheader, long width, long height,
WORD bitcount) function is an image file setting initiate function. SaveCameraToBuffer
(unsigned char * buffer) is a function that saves the screen taken by the camera current in
buffer. SaveImage (unsigned char * buff, int width, int height) Function. The screenshot ()
function is CreateCamera (0); StartPreview (); SaveCameraToBuffer (buffer); SaveImage
(buffer, 640, 480); StopPreview (); DestroyCamera (); Fb_err: close (fb_fd); By calling all of the functions, you can create a camera, put the current screen in a buffer, create an image
file, and call all functions that terminate the camera, so calling the screenshot () function
from the main function takes care of everything.
11 Touch App
Touch LCD provides a UI environment for displaying image files and enabling touch. Bmp
file to display the current status on the screen. The readFirstCoordinate function can be used
to input coordinates of the current touch point and move to the next scene. Touch each fan
or arrow to change the state of the main function to MOVING / FINISHING / WAITING
according to the characteristics of the screen. There are 6 screens in total.
12. main
Main ( ) creates Thread every components and then implement it. and also open/close the
file.
