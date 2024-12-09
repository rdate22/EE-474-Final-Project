#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // if needed for synchronization (probably not needed here)

// Define notes for Mario theme
#define E 659
#define C 523
#define G 784
#define g 392
#define R 0 // Rest

int song[] = {E, R, E, R, R, E, R, R, C, R, E, R, R, G, R, R, R, R, R, g, R};
// Adjust array length if needed

// Pin for off-board LED (choose an appropriate digital pin)
#define OFFBOARD_LED_PIN 7

// Forward declarations of tasks
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);
void TaskOffBoardLED(void *pvParameters);
void TaskMarioTheme(void *pvParameters);

void setup_timer4(int frequency) {
  // Configure Timer 4 for CTC mode toggling OC4A pin at given frequency
  // If frequency=0, stop the timer (no tone)
  if (frequency > 0) {
    TCCR4A = 0;
    TCCR4B = 0;
    // Toggle OC4A on compare match
    TCCR4A = (1 << COM4A0);
    // WGM42 set for CTC mode with OCR4A as top
    // CS41 sets prescaler = 8
    TCCR4B = (1 << WGM42) | (1 << CS41);
    OCR4A = (16000000 / (2 * 8 * frequency)) - 1; 
  } else {
    // Stop timer 4
    TCCR4A = 0;
    TCCR4B = 0;
  }
}

void setup() {
  Serial.begin(19200);
  while(!Serial) {;}
  
  // On-board LED (TaskBlink)
  pinMode(LED_BUILTIN, OUTPUT);

  // Off-board LED
  pinMode(OFFBOARD_LED_PIN, OUTPUT);

  // If using analog input pin A1 as in sample code:
  // Ensure your pot or thumbstick is connected to A1, GND, and +5V.

  // Create the original tasks
  xTaskCreate(TaskBlink, "Blink", 128, NULL, 2, NULL);
  xTaskCreate(TaskAnalogRead, "AnalogRead", 128, NULL, 1, NULL);

  // Create the new tasks
  // Task 3: Off-board LED (ON 100ms, OFF 200ms)
  xTaskCreate(TaskOffBoardLED, "OffBoardLED", 128, NULL, 1, NULL);

  // Task 4: Mario theme
  // Might need a bigger stack if we run into issues.
  xTaskCreate(TaskMarioTheme, "Mario", 256, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // empty, all tasks are handled by FreeRTOS
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskAnalogRead(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    int sensorValue = analogRead(A1); // Modify if using different pin
    Serial.println(sensorValue);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskOffBoardLED(void *pvParameters) {
  (void) pvParameters;
  // OFFBOARD_LED_PIN ON for 100ms, OFF for 200ms
  for (;;) {
    digitalWrite(OFFBOARD_LED_PIN, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(OFFBOARD_LED_PIN, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void TaskMarioTheme(void *pvParameters) {
  (void) pvParameters;

  // Play the theme 3 times
  // Each note: 100ms
  // After each full playthrough: pause 1.5s
  // After 3 times, stop the task

  int playCount = 0;

  for (;;) {
    // Play the entire song once
    for (unsigned int i = 0; i < (sizeof(song)/sizeof(int)); i++) {
      int note = song[i];
      if (note == R) {
        setup_timer4(0); // Rest
      } else {
        setup_timer4(note);
      }
      vTaskDelay(100 / portTICK_PERIOD_MS); // each note ~100ms
    }

    // Stop tone after the song
    setup_timer4(0);

    playCount++;
    if (playCount < 3) {
      // Pause 1.5s before next play
      vTaskDelay(1500 / portTICK_PERIOD_MS);
    } else {
      // After 3 plays, self-terminate
      vTaskDelete(NULL);
    }
  }
}

