#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include <queue.h>
#include <song.hpp>

#include "pcm_audio.hpp"

/*

Cher(e) stagiaire de H24,
Désolé, j'ai passé tout mon stage à prototyper le PCB et j'ai manqué de
temps pour m'occuper du logiciel. Tu devrais être pas loin de pouvoir
faire jouer quelque chose sur le haut-parleur; les librairies sont là,
mais je suis nul avec FreeRTOS... Bonne chance!

- Stagiaire de A23

*/

#define EVER \
    ;        \
    ;

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

SquareWv squarewv_;
Sawtooth sawtooth_;

struct SwitchData
{
    bool sw1;
    bool sw2;
} sSwitchData;

struct PotentiometerData
{
    float tempo;
    float vcaLength;
    float coupure;
    float frequence;
} sPotentiometerData;

struct FilterCoeffs
{
    float b1;
    float a1;
    float a2;
} sFilterCoeffs;

QueueHandle_t switchDataQueue;
QueueHandle_t potDataQueue;
QueueHandle_t filterCoeffsQueue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCF(int8_t input)
{
    static int8_t y1 = 0;
    static int8_t y2 = 0;

    if (xQueueReceive(filterCoeffsQueue, &sFilterCoeffs, 0)) // Non-blocking check
    {
    }

    float b1 = sFilterCoeffs.b1;
    float a1 = sFilterCoeffs.a1;
    float a2 = sFilterCoeffs.a2;

    // Serial.println(b1);
    // Serial.println(a1);
    // Serial.println(a2);

    int16_t y0 = b1 * input + a1 * y1 + a2 * y2;

    int8_t output = 0xFF & (y0 >> 8);

    y2 = y1;
    y1 = output;

    return output;
}

int8_t processVCA(int8_t input)
{
    if (xQueueReceive(potDataQueue, &sPotentiometerData, 0)) {}
    if (xQueueReceive(switchDataQueue, &sSwitchData, 0)) {}

    float bufferIterations = 3 * 8000;  // 3 seconds * 8000 samples/second
    static float iteration = bufferIterations;  // Start at end of decay
    int8_t frequencyOutput;

    if (sSwitchData.sw1)
    {
        iteration = 0;  // Reset when button is pressed
        frequencyOutput = input;
    }
    else
    {
        // Linear decay over 3 seconds
        if (iteration < bufferIterations)
        {
            float decay = 1.0f - (iteration / bufferIterations);
            frequencyOutput = input * decay;
            iteration++;
        }
        else
        {
            frequencyOutput = 0;  // Complete silence after 3 seconds
        }
    }

    return frequencyOutput;
}
int8_t nextSample()
{
    int8_t vco = sawtooth_.next() + squarewv_.next();
    int8_t vcf = processVCF(vco);
    int8_t vca = processVCA(vcf);

    int8_t output = vca;

    return output;
}

// Fonctions ISR
void sw1Isr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    sSwitchData.sw1 = digitalRead(PIN_SW1);

    xQueueOverwriteFromISR(switchDataQueue, &sSwitchData, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void sw2Isr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    sSwitchData.sw2 = digitalRead(PIN_SW2);

    xQueueOverwriteFromISR(switchDataQueue, &sSwitchData, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void readPotentiometer(void *pvParameters __attribute__((unused)))
{
    for (EVER)
    {
        sPotentiometerData.tempo = (analogRead(PIN_RV1) / 1080.0f) * (240.0f - 60.0f) + 240.0f;
        sPotentiometerData.vcaLength = (analogRead(PIN_RV2) / 3.0f) * 1023.0f;
        sPotentiometerData.coupure = (analogRead(PIN_RV3) * PI) / 1023.0f;
        sPotentiometerData.frequence = analogRead(PIN_RV4) / 1023.0f;

        float f = sPotentiometerData.coupure;
        float q = sPotentiometerData.frequence;

        float fb = (q + (q / (1.0f - f)));
        sFilterCoeffs.b1 = f * f * 256.0f;
        sFilterCoeffs.a1 = (2 - 2 * f + f * fb - f * f * fb) * 256.0f;
        sFilterCoeffs.a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256.0f;

        xQueueOverwrite(potDataQueue, &sPotentiometerData);
        xQueueOverwrite(filterCoeffsQueue, &sFilterCoeffs);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void fillBuffer(void *pvParameters __attribute__((unused)))
{
    for (EVER)
    {
        if (!pcmBufferFull())
        {
            pcmAddSample(nextSample());
        }
        else
        {
            taskYIELD();
        }
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), sw1Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), sw2Isr, CHANGE);

    Serial.begin(9600);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);

    switchDataQueue = xQueueCreate(1, sizeof(SwitchData));
    potDataQueue = xQueueCreate(1, sizeof(PotentiometerData));
    filterCoeffsQueue = xQueueCreate(1, sizeof(FilterCoeffs));

    setNoteHz(440.0);

    pcmSetup();

    xTaskCreate(
        fillBuffer,
        "buttonTask",
        256,
        NULL,
        2,
        NULL);

    xTaskCreate(
        readPotentiometer,
        "readPotentiometer",
        256,
        NULL,
        3,
        NULL);
}

void loop()
{
}