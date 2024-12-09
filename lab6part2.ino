#include <Arduino_FreeRTOS.h>
#include <semphr.h> 
#include <string.h>
#include <stdlib.h> // for rand()

// ------------------- Configuration -------------------
#define LED_BUILTIN_PIN 13     // On-board LED
#define OFFBOARD_LED_PIN 7     // Off-board LED pin
#define JOYSTICK_X_PIN A1      // Joystick X
#define JOYSTICK_Y_PIN A2      // Joystick Y

// Matrix Pins (adjust these to your wiring)
int rowPins[8] = {2,3,4,5,6,7,8,9};
int colPins[8] = {10,11,12,13,A0,A3,A4,A5};
// NOTE: Adjust pins as needed.

#define MATRIX_SIZE 8

// Task configurations
#define STACK_BLINK 128
#define STACK_ANALOGREAD 128
#define STACK_OFFBOARD_LED 128
#define STACK_SNAKEGAME 256
#define STACK_MATRIXDISPLAY 256
#define STACK_MATRIXSCAN 256

// Timing intervals
#define BLINK_ON_MS 250
#define BLINK_OFF_MS 100
#define ANALOG_READ_MS 50
#define SNAKE_UPDATE_MS 20     // 50 Hz update
#define MATRIX_UPDATE_MS 20    // Also 50 Hz
#define MATRIX_SCAN_DELAY_US 2000 // per row scanning delay

// Snake game definitions
struct SnakeSegment {
  int x,y;
};
#define MAX_SNAKE_LENGTH 30

static SnakeSegment snake[MAX_SNAKE_LENGTH];
static int snakeLength = 3;
static int snakeDirX = 1; // start moving right
static int snakeDirY = 0;
static int snakeHeadX = 3, snakeHeadY = 3;
static bool gameOver = false;

static int foodX, foodY;
static bool hasFood = false;

static int joyX = 512;
static int joyY = 512;

static bool matrixBuffer[8][8]; // Frame buffer for LED matrix

// Forward declarations
void TaskBlink(void *pvParameters);
void TaskAnalogReadJoystick(void *pvParameters);
void TaskOffBoardLED(void *pvParameters);
void TaskSnakeGame(void *pvParameters);
void TaskMatrixDisplay(void *pvParameters);
void TaskMatrixScan(void *pvParameters);

// Helpers
void matrixInit();
void matrixClear();
void matrixSetPixel(int x, int y, bool on);
void matrixScanDisplay();
void snakeInit();
void updateDirectionFromJoystick();
void moveSnake();
void placeFood();
void drawSnakeAndFood();
void checkCollision();
void growSnake();

// ------------------- SETUP & LOOP -------------------
void setup() {
  Serial.begin(19200);
  while(!Serial){;}

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  pinMode(OFFBOARD_LED_PIN, OUTPUT);

  matrixInit();
  matrixClear();

  snakeInit();
  placeFood();

  // Create tasks
  xTaskCreate(TaskBlink, "Blink", STACK_BLINK, NULL, 2, NULL);
  xTaskCreate(TaskAnalogReadJoystick, "AnalogRead", STACK_ANALOGREAD, NULL, 1, NULL);
  xTaskCreate(TaskOffBoardLED, "OffBoardLED", STACK_OFFBOARD_LED, NULL, 1, NULL);
  xTaskCreate(TaskSnakeGame, "SnakeGame", STACK_SNAKEGAME, NULL, 3, NULL);
  xTaskCreate(TaskMatrixDisplay, "MatrixDisp", STACK_MATRIXDISPLAY, NULL, 1, NULL);
  xTaskCreate(TaskMatrixScan, "MatrixScan", STACK_MATRIXSCAN, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Empty, all handled by tasks
}

// ------------------- SNAKE GAME LOGIC -------------------
void snakeInit() {
  snakeLength = 3;
  snakeHeadX = MATRIX_SIZE/2;
  snakeHeadY = MATRIX_SIZE/2;
  for (int i=0; i<snakeLength; i++){
    snake[i].x = snakeHeadX - i;
    snake[i].y = snakeHeadY;
  }
  snakeDirX = 1; snakeDirY = 0;
  gameOver = false;
}

void placeFood() {
  // Place food in a random location not on the snake
  // Simple approach: try random positions until we find a free spot
  int tries = 0;
  do {
    foodX = random(0,8);
    foodY = random(0,8);
    bool onSnake = false;
    for (int i=0; i<snakeLength; i++) {
      if (snake[i].x == foodX && snake[i].y == foodY) {
        onSnake = true; break;
      }
    }
    if (!onSnake) {
      hasFood = true;
      return;
    }
    tries++;
  } while (tries < 100);

  // If we fail to place after 100 tries (unlikely), no food
  hasFood = false;
}

void updateDirectionFromJoystick() {
  int center = 512;
  int threshold = 200;
  int dx = joyX - center;
  int dy = joyY - center;

  // favor horizontal or vertical based on which is larger
  if (abs(dx) > abs(dy)) {
    // horizontal
    if (dx > threshold) {
      // move right
      if (!(snakeDirX == -1 && snakeDirY == 0)) { snakeDirX=1; snakeDirY=0; }
    } else if (dx < -threshold) {
      // move left
      if (!(snakeDirX == 1 && snakeDirY==0)) { snakeDirX=-1; snakeDirY=0; }
    }
  } else {
    // vertical
    if (dy > threshold) {
      // move down
      if (!(snakeDirX==0 && snakeDirY==-1)) { snakeDirX=0; snakeDirY=1; }
    } else if (dy < -threshold) {
      // move up
      if (!(snakeDirX==0 && snakeDirY==1)) { snakeDirX=0; snakeDirY=-1; }
    }
  }
}

void moveSnake() {
  if (gameOver) return;

  snakeHeadX += snakeDirX;
  snakeHeadY += snakeDirY;

  // Check boundaries
  if (snakeHeadX<0||snakeHeadX>=MATRIX_SIZE||snakeHeadY<0||snakeHeadY>=MATRIX_SIZE) {
    gameOver=true;return;
  }

  // Check self collision
  for (int i=0; i<snakeLength;i++){
    if (snake[i].x==snakeHeadX && snake[i].y==snakeHeadY){
      gameOver=true;return;
    }
  }

  // Move body
  for (int i=snakeLength-1;i>0;i--){
    snake[i]=snake[i-1];
  }
  snake[0].x=snakeHeadX;
  snake[0].y=snakeHeadY;

  // Check food
  if (hasFood && snakeHeadX==foodX && snakeHeadY==foodY) {
    // eat and grow
    growSnake();
    hasFood=false;
    placeFood();
  }
}

void growSnake() {
  if (snakeLength<MAX_SNAKE_LENGTH) {
    // add a segment at the end, same pos as last tail (just grows in place)
    snake[snakeLength] = snake[snakeLength-1];
    snakeLength++;
  }
}

void drawSnakeAndFood() {
  matrixClear();
  if (gameOver) {
    // Show all LEDs ON for game over
    for (int x=0;x<MATRIX_SIZE;x++){
      for (int y=0;y<MATRIX_SIZE;y++){
        matrixSetPixel(x,y,true);
      }
    }
  } else {
    // draw snake
    for (int i=0;i<snakeLength;i++){
      matrixSetPixel(snake[i].x,snake[i].y,true);
    }
    // draw food
    if (hasFood) {
      matrixSetPixel(foodX,foodY,true);
    }
  }
}

// ------------------- MATRIX CONTROL -------------------
void matrixInit() {
  // Set rows and columns as outputs
  for (int r=0;r<8;r++){
    pinMode(rowPins[r],OUTPUT);
    digitalWrite(rowPins[r],HIGH); // active LOW rows
  }
  for (int c=0;c<8;c++){
    pinMode(colPins[c],OUTPUT);
    digitalWrite(colPins[c],LOW); // assume columns active HIGH
  }
  matrixClear();
}

// Clear the buffer
void matrixClear() {
  for (int y=0;y<8;y++){
    for (int x=0;x<8;x++){
      matrixBuffer[y][x]=false;
    }
  }
}

void matrixSetPixel(int x,int y,bool on) {
  if (x<0||x>7||y<0||y>7)return;
  matrixBuffer[y][x]=on;
}

void matrixScanDisplay() {
  // Direct drive scanning
  // Turn all rows off first
  for (int rr=0;rr<8;rr++){
    digitalWrite(rowPins[rr],HIGH);
  }

  // Scan each row
  for (int r=0;r<8;r++){
    // Set columns for this row
    for (int c=0;c<8;c++){
      if (matrixBuffer[r][c]) {
        digitalWrite(colPins[c],HIGH); // LED on
      } else {
        digitalWrite(colPins[c],LOW);
      }
    }

    // Enable this row (active LOW)
    digitalWrite(rowPins[r],LOW);
    delayMicroseconds(MATRIX_SCAN_DELAY_US);
    digitalWrite(rowPins[r],HIGH);
  }
}

// ------------------- TASKS -------------------

void TaskBlink(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(LED_BUILTIN_PIN,HIGH);
    vTaskDelay(BLINK_ON_MS / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN_PIN,LOW);
    vTaskDelay(BLINK_OFF_MS / portTICK_PERIOD_MS);
  }
}

void TaskAnalogReadJoystick(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    joyX=analogRead(JOYSTICK_X_PIN);
    joyY=analogRead(JOYSTICK_Y_PIN);
    vTaskDelay(ANALOG_READ_MS/portTICK_PERIOD_MS);
  }
}

void TaskOffBoardLED(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(OFFBOARD_LED_PIN,HIGH);
    vTaskDelay(100/portTICK_PERIOD_MS);
    digitalWrite(OFFBOARD_LED_PIN,LOW);
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}

void TaskSnakeGame(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (!gameOver) {
      updateDirectionFromJoystick();
      moveSnake();
    }
    vTaskDelay(SNAKE_UPDATE_MS / portTICK_PERIOD_MS);
  }
}

void TaskMatrixDisplay(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    drawSnakeAndFood();
    // No hardware update here, just refresh the buffer
    // The actual LED lighting is done by the scanning task
    vTaskDelay(MATRIX_UPDATE_MS / portTICK_PERIOD_MS);
  }
}

void TaskMatrixScan(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    matrixScanDisplay();
    // No delay needed because scanning includes a delay per row
    // You can add a small delay if flicker or brightness is off
  }
}

