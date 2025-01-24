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

TaskHandle_t buttonTaskHandle_ = NULL;
TaskHandle_t bufferTaskHandle = NULL;

QueueHandle_t freqQueue;
QueueHandle_t tempoQueue;
QueueHandle_t coupureQueue;
QueueHandle_t vcaLengthQueue;

EventGroupHandle_t eventGroup_;

// float f = 1.0;
// float q = 0.5;
// float fb = (q + (q / (1.0 - f)));
// int16_t b1 = f * f * 256;
// int16_t a1 = (2 - 2 * f + f * fb - f * f * fb) * 256;
// int16_t a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256;

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

int8_t nextSample()
{
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();

    // VCF (enabled)
    int8_t vcf = vco;

    // VCA (disabled)
    int8_t vca = vcf;

    int8_t output = vca;

    return output;
}

void buffer(void *pvParameters __attribute__((unused)))
{
    for (EVER)
    {
        vTaskSuspend(NULL);

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
        xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);

        if (notificationValue == PIN_SW1)
        {
            if (digitalRead(PIN_SW1) == HIGH)
            {
                // Serial.println("Button 1 pressed");
                setNoteHz(440.0f);
                pcmAddSample(nextSample());
            }
            else
            {
                // Serial.println("Button 1 unpressed");
                setNoteHz(0.0f);
                pcmAddSample(nextSample());
            }
        }

        // if (notificationValue == PIN_SW2)
        // {
        //     if (digitalRead(PIN_SW2) == HIGH)
        //     {
        //         // Serial.println("Button pressed");

        //         // if (!pcmBufferFull())
        //         // {
        //         //     pcmAddSample(nextSample());
        //         // }
        //         // else
        //         // {
        //         //     taskYIELD();
        //         // }
        //     }
        //     else
        //     {
        //         // Serial.println("Button 2 not pressed");
        //         // setNoteHz(0.0f);
        //         // pcmAddSample(nextSample());
        //     }
        // }

        taskYIELD();
    }
}

void readPotentiometer(void *pvParameters __attribute__((unused)))
{
    for (EVER)
    {
        float tempo = (analogRead(PIN_RV1) / 1080.0f) * (240.0f - 60.0f) + 240.0f;
        xQueueOverwrite(tempoQueue, &tempo);
        float vcaLength = (analogRead(PIN_RV2) * 3.0f) / 1023.0f;
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
        buttonTaskHandle_,
        PIN_SW1,
        eSetValueWithOverwrite,
        &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
}

void sw2Isr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
        buttonTaskHandle_,
        PIN_SW2,
        eSetValueWithOverwrite,
        &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
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

    eventGroup_ = xEventGroupCreate();

    xTaskCreate(
        buttonTask,
        "buttonTask",
        256,
        NULL,
        3,
        &buttonTaskHandle_);

    xTaskCreate(
        buffer,
        "buttonTask",
        256,
        NULL,
        0,
        &bufferTaskHandle);

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
