#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <string.h>
#include <stdlib.h> // for rand()

// ------------------- Configuration -------------------
// Matrix SPI pins
#define DIN_PIN 6  // Data In
#define CLK_PIN 7  // Clock
#define CS_PIN 5   // Chip Select

// Matrix dimensions
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8

// Joystick Pins
#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1

// Task configurations
#define STACK_BLINK 128
#define STACK_ANALOGREAD 128
#define STACK_SNAKEGAME 256
#define STACK_MATRIXDISPLAY 256

// Timing intervals
#define BLINK_ON_MS 250
#define BLINK_OFF_MS 100
#define ANALOG_READ_MS 50
#define SNAKE_UPDATE_MS 500     // Slower update: 2 Hz
#define MATRIX_UPDATE_MS 50     // 20 Hz update

// Snake game definitions
struct SnakeSegment {
  int x, y;
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

static byte matrixBuffer[8]; // 8 bytes for an 8x8 matrix

// Forward declarations
void TaskBlink(void *pvParameters);
void TaskAnalogReadJoystick(void *pvParameters);
void TaskSnakeGame(void *pvParameters);
void TaskMatrixDisplay(void *pvParameters);

// Helpers
void matrixInit();
void matrixClear();
void matrixSetPixel(int x, int y, bool on);
void matrixSendData(byte address, byte data);
void matrixSendBuffer();
void snakeInit();
void updateDirectionFromJoystick();
void moveSnake();
void placeFood();
void drawSnakeAndFood();
void growSnake();

// ------------------- SETUP & LOOP -------------------
void setup() {
  Serial.begin(19200);
  while (!Serial) {;}

  pinMode(LED_BUILTIN, OUTPUT);

  matrixInit();
  snakeInit();
  placeFood();

  // Create tasks
  xTaskCreate(TaskBlink, "Blink", STACK_BLINK, NULL, 2, NULL);
  xTaskCreate(TaskAnalogReadJoystick, "AnalogRead", STACK_ANALOGREAD, NULL, 1, NULL);
  xTaskCreate(TaskSnakeGame, "SnakeGame", STACK_SNAKEGAME, NULL, 3, NULL);
  xTaskCreate(TaskMatrixDisplay, "MatrixDisplay", STACK_MATRIXDISPLAY, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Empty, all handled by tasks
}

// ------------------- MATRIX CONTROL -------------------
void matrixInit() {
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  digitalWrite(CS_PIN, HIGH); // Disable matrix communication
  matrixClear();

  // Initialize the MAX7219 registers
  matrixSendData(0x09, 0x00); // Decode mode: None
  matrixSendData(0x0A, 0x08); // Intensity: Medium
  matrixSendData(0x0B, 0x07); // Scan limit: All rows
  matrixSendData(0x0C, 0x01); // Shutdown: Normal operation
  matrixSendData(0x0F, 0x00); // Display test: Off
}

void matrixClear() {
  memset(matrixBuffer, 0, sizeof(matrixBuffer));
  matrixSendBuffer();
}

void matrixSetPixel(int x, int y, bool on) {
  if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
  if (on)
    matrixBuffer[y] |= (1 << x);
  else
    matrixBuffer[y] &= ~(1 << x);
}

void matrixSendData(byte address, byte data) {
  digitalWrite(CS_PIN, LOW); // Enable communication
  shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, address);
  shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, data);
  digitalWrite(CS_PIN, HIGH); // Disable communication
}

void matrixSendBuffer() {
  for (byte row = 0; row < 8; row++) {
    matrixSendData(row + 1, matrixBuffer[row]);
  }
}

// ------------------- SNAKE GAME LOGIC -------------------
void snakeInit() {
  snakeLength = 3;
  snakeHeadX = MATRIX_WIDTH / 2;
  snakeHeadY = MATRIX_HEIGHT / 2;
  for (int i = 0; i < snakeLength; i++) {
    snake[i].x = snakeHeadX - i;
    snake[i].y = snakeHeadY;
  }
  snakeDirX = 1; snakeDirY = 0;
  gameOver = false;
}

void placeFood() {
  int tries = 0;
  do {
    foodX = random(0, MATRIX_WIDTH);
    foodY = random(0, MATRIX_HEIGHT);
    bool onSnake = false;
    for (int i = 0; i < snakeLength; i++) {
      if (snake[i].x == foodX && snake[i].y == foodY) {
        onSnake = true;
        break;
      }
    }
    if (!onSnake) {
      hasFood = true;
      return;
    }
    tries++;
  } while (tries < 100);
  hasFood = false;
}

void updateDirectionFromJoystick() {
  if (gameOver) return;

  int center = 512;
  int threshold = 200;
  int dx = joyX - center;
  int dy = joyY - center;

  if (abs(dx) > abs(dy)) {
    if (dx > threshold && !(snakeDirX == -1 && snakeDirY == 0)) { snakeDirX = 1; snakeDirY = 0; }
    else if (dx < -threshold && !(snakeDirX == 1 && snakeDirY == 0)) { snakeDirX = -1; snakeDirY = 0; }
  } else {
    if (dy > threshold && !(snakeDirX == 0 && snakeDirY == -1)) { snakeDirX = 0; snakeDirY = 1; }
    else if (dy < -threshold && !(snakeDirX == 0 && snakeDirY == 1)) { snakeDirX = 0; snakeDirY = -1; }
  }
}

void moveSnake() {
  if (gameOver) return;

  snakeHeadX += snakeDirX;
  snakeHeadY += snakeDirY;

  if (snakeHeadX < 0 || snakeHeadX >= MATRIX_WIDTH || snakeHeadY < 0 || snakeHeadY >= MATRIX_HEIGHT) {
    gameOver = true;
    return;
  }

  for (int i = 0; i < snakeLength; i++) {
    if (snake[i].x == snakeHeadX && snake[i].y == snakeHeadY) {
      gameOver = true;
      return;
    }
  }

  for (int i = snakeLength - 1; i > 0; i--) {
    snake[i] = snake[i - 1];
  }
  snake[0].x = snakeHeadX;
  snake[0].y = snakeHeadY;

  if (hasFood && snakeHeadX == foodX && snakeHeadY == foodY) {
    growSnake();
    hasFood = false;
    placeFood();
  }
}

void growSnake() {
  if (snakeLength < MAX_SNAKE_LENGTH) {
    snake[snakeLength] = snake[snakeLength - 1];
    snakeLength++;
  }
}

void drawSnakeAndFood() {
  matrixClear();
  for (int i = 0; i < snakeLength; i++) {
    matrixSetPixel(snake[i].x, snake[i].y, true);
  }
  if (hasFood && !gameOver) {
    matrixSetPixel(foodX, foodY, true);
  }
}

// ------------------- TASKS -------------------
void TaskBlink(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(BLINK_ON_MS / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(BLINK_OFF_MS / portTICK_PERIOD_MS);
  }
}

void TaskAnalogReadJoystick(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    joyX = analogRead(JOYSTICK_X_PIN);
    joyY = analogRead(JOYSTICK_Y_PIN);
    vTaskDelay(ANALOG_READ_MS / portTICK_PERIOD_MS);
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
    matrixSendBuffer();
    vTaskDelay(MATRIX_UPDATE_MS / portTICK_PERIOD_MS);
  }
}
