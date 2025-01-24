#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include <event_groups.h>
#include <queue.h>

#include "pcm_audio.hpp"

/*

Cher(e) stagiaire de H24,
Désolé, j'ai passé tout mon stage à prototyper le PCB et j'ai manqué de
temps pour m'occuper du logiciel. Tu devrais être pas loin de pouvoir
faire jouer quelque chose sur le haut-parleur; les librairies sont là,
mais je suis nul avec FreeRTOS... Bonne chance!

- Stagiaire de A23

*/

// Use queue for F and Q variable

#define EVER \
    ;        \
    ;

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SIG 22
#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

SquareWv squarewv_;
Sawtooth sawtooth_;

TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t bufferTaskHandle = NULL;

QueueHandle_t freqQueue;
QueueHandle_t tempoQueue;
QueueHandle_t coupureQueue;
QueueHandle_t vcaLengthQueue;

QueueHandle_t sw1Queue;
QueueHandle_t sw2Queue;

uint32_t notifValue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t processVCF(int8_t input)
{
    float f;
    float q;

    xQueuePeek(coupureQueue, &f, portMAX_DELAY);
    xQueuePeek(freqQueue, &q, portMAX_DELAY);

    float fb = (q + (q / (1.0 - f)));
    int16_t b1 = f * f * 256;
    int16_t a1 = (2 - 2 * f + f * fb - f * f * fb) * 256;
    int16_t a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256;

    static int8_t y1 = 0;
    static int8_t y2 = 0;

    int16_t y0 = b1 * input + a1 * y1 + a2 * y2;

    int8_t output = 0xFF & (y0 >> 8);

    y2 = y1;
    y1 = output;

    return output;
}

int8_t processVca(int8_t input)
{
    float currentVcaLength;
    xQueuePeek(vcaLengthQueue, &currentVcaLength, portMAX_DELAY);
    float bufferIterations = currentVcaLength * SAMPLE_RATE;

    static float iteration = bufferIterations;
    int8_t frequencyOutput;

    int sw1State;
    int sw2State;

    xQueuePeek(sw1Queue, &sw1State, portMAX_DELAY);
    xQueuePeek(sw2Queue, &sw2State, portMAX_DELAY);

    if (sw1State)
    {
        frequencyOutput = input;
        iteration = 1.0f;

    }
    else
    {
        float output_down = -iteration * input / bufferIterations + input;
        frequencyOutput = output_down;

        if (iteration >= bufferIterations)
        {
            iteration = bufferIterations;
        }
        else
        {
            iteration++;
        }
    }

    return frequencyOutput;
}

int8_t nextSample()
{
    int8_t vco = sawtooth_.next() + squarewv_.next();

    int8_t vcf = processVCF(vco);

    int8_t vca = processVca(vcf);

    int8_t output = vca;

    return output;
}

void buffer(void *pvParameters __attribute__((unused)))
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

void buttonTask(void *pvParameters __attribute__((unused)))
{
    uint32_t notificationValue;

    for (EVER)
    {
        xTaskNotifyWait(0, 0, &notificationValue, portMAX_DELAY);

        int sw1PinState = digitalRead(PIN_SW1);
        int sw2PinState = digitalRead(PIN_SW2);

        xQueueOverwrite(sw1Queue, &sw1PinState);
        xQueueOverwrite(sw2Queue, &sw2PinState);
    }
}

void readPotentiometer(void *pvParameters __attribute__((unused)))
{
    for (EVER)
    {
        float tempo = (analogRead(PIN_RV1) / 1080.0f) * (240.0f - 60.0f) + 240.0f;
        xQueueOverwrite(tempoQueue, &tempo);
        float vcaLength = (analogRead(PIN_RV2) / 1023.0f) * 3.0f;
        xQueueOverwrite(vcaLengthQueue, &vcaLength);
        float coupure = (analogRead(PIN_RV3) * PI) / 1023.0f;
        xQueueOverwrite(coupureQueue, &coupure);
        float frequence = analogRead(PIN_RV4) / 1023.0f;
        xQueueOverwrite(freqQueue, &frequence);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void sw1Isr()
{
    // In this case, eSetValueWithOverwrite means if multiple button presses occur before the task processes the first one, only the most recent notification will be kept.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
        buttonTaskHandle,
        PIN_SW1,
        eSetValueWithOverwrite,
        &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void sw2Isr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
        buttonTaskHandle,
        PIN_SW2,
        eSetValueWithOverwrite,
        &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);
    pinMode(PIN_SW2, INPUT);
    pinMode(PIN_SIG, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), sw1Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), sw2Isr, CHANGE);

    Serial.begin(9600);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(440.0);

    pcmSetup();

    freqQueue = xQueueCreate(1, sizeof(float));
    tempoQueue = xQueueCreate(1, sizeof(float));
    coupureQueue = xQueueCreate(1, sizeof(float));
    vcaLengthQueue = xQueueCreate(1, sizeof(float));

    sw1Queue = xQueueCreate(1, sizeof(int));
    sw2Queue = xQueueCreate(1, sizeof(int));


    xTaskCreate(
        buttonTask,
        "buttonTask",
        256,
        NULL,
        2,
        &buttonTaskHandle);

    xTaskCreate(
        buffer,
        "buttonTask",
        256,
        NULL,
        1,
        &bufferTaskHandle);

    xTaskCreate(
        readPotentiometer,
        "readPotentiometer",
        256,
        NULL,
        2,
        NULL);
}

void loop()
{
}
