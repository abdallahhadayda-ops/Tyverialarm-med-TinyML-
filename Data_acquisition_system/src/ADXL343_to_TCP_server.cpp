/**
 * @file ADXL343_to_TCP_server.cpp
 * @author morten opprud
 * @brief
 * @version 0.1
 * @date 2024-12-16
 */

#include "Particle.h"
#include "adxl343.h" // Include the ADXL343 driver

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

// Constants
const unsigned long MAX_RECORDING_LENGTH_MS = 30000; // 30000;
const int SAMPLING_INTERVAL_MS = 2;                 // Sampling interval in milliseconds
const int BUFFER_SIZE = 25;                          // Buffer size
const int TRANSMIT_THRESHOLD = 20;                   // Transmit buffer after 20 samples

// Server configuration
IPAddress serverAddr = IPAddress( 10,112,7,107);
int serverPort = 7123;

TCPClient client;

// Buffers for double buffering
struct Sample
{
  uint32_t timestamp; // Timestamp in microseconds
  float x, y, z;      // Accelerometer data
};

Sample buffer1[BUFFER_SIZE];
Sample buffer2[BUFFER_SIZE];
Sample *samplingBuffer = buffer1;
Sample *transmitBuffer = buffer2;
volatile int samplingIndex = 0;
volatile bool bufferReady = false;

// ADXL343 accelerometer
ADXL343 accelerometer;

void sampleAccelerometer();
void transmitBufferData();
void buttonHandler(system_event_t event, int data);

// Timer for sampling
Timer samplingTimer(SAMPLING_INTERVAL_MS, sampleAccelerometer);

// State machine
enum State
{
  STATE_WAITING,
  STATE_CONNECT,
  STATE_RUNNING,
  STATE_FINISH
};
State state = STATE_WAITING;

unsigned long recordingStart = 0;
unsigned long lastSampleTime = 0;

#define LED_PIN D7

void buttonHandler(system_event_t event, int data);
void transmitBufferData(int samples);

void setup()
{
  Particle.connect();
  System.on(button_click, buttonHandler);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (!accelerometer.begin())
  {
    Log.error("Failed to initialize ADXL343!");
    System.reset();
  }

  Log.info("ADXL343 initialized. Waiting for button press...");
}

void loop()
{
  switch (state)
  {
  case STATE_WAITING:
    break;

  case STATE_CONNECT:
    if (client.connect(serverAddr, serverPort))
    {
      Log.info("Connected to server. Starting data collection...");
      Log.info("Sample rate: %d Hz", 1000 / SAMPLING_INTERVAL_MS);
      recordingStart = millis();
      samplingIndex = 0;
      samplingTimer.start();
      digitalWrite(LED_PIN, HIGH);
      state = STATE_RUNNING;
    }
    else
    {
      Log.error("Failed to connect to server.");
      state = STATE_WAITING;
    }
    break;

  case STATE_RUNNING:
    // Check for buffer ready to transmit
    if (bufferReady)
    {
      // Swap buffers before transmission
      Sample *temp = samplingBuffer;
      samplingBuffer = transmitBuffer;
      transmitBuffer = temp;

      int samplesToTransmit = samplingIndex; // Snapshot of current index
      samplingIndex = 0;                     // Reset sampling index
      bufferReady = false;

      // Transmit the previous buffer
      transmitBufferData(samplesToTransmit);
    }

    // Stop after timeout
    if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS)
    {
      state = STATE_FINISH;
    }
    break;

  case STATE_FINISH:
    samplingTimer.stop();
    if (samplingIndex > 0)
    {
      bufferReady = true;
      transmitBufferData(samplingIndex);
    }
    client.stop();
    digitalWrite(LED_PIN, LOW);
    Log.info("Data collection complete.");
    state = STATE_WAITING;
    break;
  }
}

void sampleAccelerometer()
{
  if (samplingIndex < BUFFER_SIZE)
  {
    // Sample accelerometer
    float x, y, z;
    accelerometer.readAccelerationG(&x, &y, &z);

    samplingBuffer[samplingIndex].timestamp = micros();
    samplingBuffer[samplingIndex].x = x;
    samplingBuffer[samplingIndex].y = y;
    samplingBuffer[samplingIndex].z = z;

    samplingIndex++;
    lastSampleTime = millis();

    // Mark buffer as ready if threshold is met
    if (samplingIndex >= TRANSMIT_THRESHOLD)
    {
      bufferReady = true;
    }
  }
}

void transmitBufferData(int samples)
{
  if (samples == 0)
  {
    Log.warn("Transmit called with empty buffer. Skipping transmission.");
    return;
  }
  //Log.info("Transmitting %d samples...", samples);
  for (int i = 0; i < samples; i++)
  {
    String data = String::format(
        "%lu,%.2f,%.2f,%.2f\n",
        transmitBuffer[i].timestamp,
        transmitBuffer[i].x,
        transmitBuffer[i].y,
        transmitBuffer[i].z);
    client.write(((const uint8_t *)data.c_str()), data.length());
  }
  //Log.info("Buffer transmission complete.");
}

void buttonHandler(system_event_t event, int data)
{
  switch (state)
  {
  case STATE_WAITING:
    if (WiFi.ready())
    {
      state = STATE_CONNECT;
    }
    else
    {
      Log.warn("Wi-Fi not ready.");
    }
    break;

  case STATE_RUNNING:
    state = STATE_FINISH;
    break;
  }
}

#if 0
#include "Particle.h"
#include "adxl343.h" // Include the ADXL343 driver

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

// Maximum data collection duration in milliseconds
const unsigned long MAX_RECORDING_LENGTH_MS = 30000;

// Server configuration
IPAddress serverAddr = IPAddress( 10,112,7,107); // **UPDATE THIS**
int serverPort = 7123;

TCPClient client;
unsigned long recordingStart;

// State machine
enum State
{
  STATE_WAITING,
  STATE_CONNECT,
  STATE_RUNNING,
  STATE_FINISH
};
State state = STATE_WAITING;

// ADXL343 accelerometer instance
ADXL343 accelerometer;

// Button handling
void buttonHandler(system_event_t event, int data);

// LED setup
#define LED_PIN D7

void setup()
{
  // Enable Wi-Fi connection
  Particle.connect();

  // Register button handler
  System.on(button_click, buttonHandler);

  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize ADXL343
  if (!accelerometer.begin())
  {
    Log.error("Failed to initialize ADXL343 accelerometer!");
    System.reset(); // Restart if initialization fails
  }
  Log.info("ADXL343 initialized successfully.");
  Log.info("Waiting for button press to start data collection...");

  // Log device IP address
  Log.info("Device IP Address: %s", WiFi.localIP().toString().c_str());
}

void loop()
{
  float accX, accY, accZ;
  String data;

  switch (state)
  {
    case STATE_WAITING:
      // Waiting for button press (handled in buttonHandler)
      break;

    case STATE_CONNECT:
      // Attempt to connect to the server
      if (client.connect(serverAddr, serverPort))
      {
        Log.info("Connected to server. Starting data collection.");
        recordingStart = millis();
        digitalWrite(LED_PIN, HIGH);
        state = STATE_RUNNING;
      }
      else
      {
        Log.error("Failed to connect to server.");
        state = STATE_WAITING;
      }
      break;

    case STATE_RUNNING:
      // Read accelerometer data and send to server
      accelerometer.readAccelerationG(&accX, &accY, &accZ);

      // Format data as "timestamp,x,y,z\n"
      data = String::format("%lu,%.2f,%.2f,%.2f\n", millis(), accX, accY, accZ);
      // client.write(data.c_str(), data.length());
      client.write(((const uint8_t *)data.c_str()), data.length());

      // Stop after the maximum recording length
      if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS)
      {
        state = STATE_FINISH;
      }
      break;

    case STATE_FINISH:
      // Stop data collection and close connection
      digitalWrite(LED_PIN, LOW);
      client.stop();
      Log.info("Stopped data collection. Returning to waiting state.");
      state = STATE_WAITING;
      break;
  }
}

void buttonHandler(system_event_t event, int data)
{
  switch (state)
  {
  case STATE_WAITING:
    if (WiFi.ready())
    {
      Log.info("Button pressed. Connecting to server...");
      state = STATE_CONNECT;
    }
    else
    {
      Log.warn("Wi-Fi not ready. Cannot start data collection.");
    }
    break;

  case STATE_RUNNING:
    Log.info("Button pressed. Stopping data collection...");
    state = STATE_FINISH;
    break;
  }
}
#endif