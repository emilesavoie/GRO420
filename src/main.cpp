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

/*
Ce define a pour utilité unique de clarifier la lecture du code
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

/*
Les structs suivantes sont définies afin de limiter la création et la gestion de queues dans le programme.
De cette manière, il est possible d'éviter le overhead non nécéssaire. Les structs sont séparés selon des
groupes déterminés. Toutes les valeurs auraient pu être mises dans une seule queues, mais cela nuirait à
la lisibilité ainsi qu'à la modularité. Des objets de queues sont ensuite créer pour chacune des structs
*/

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

struct MelodyData
{
    int currentNoteIndex = 0;
    unsigned long noteStartTime = 0;
    bool playSong = false;
} sMelodyData;

QueueHandle_t switchDataQueue;
QueueHandle_t potDataQueue;
QueueHandle_t filterCoeffsQueue;
QueueHandle_t melodyDataQueue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

/*
La fonction processVCF demeure presque identique à l'exeption qu'une partie des calculs se trouvent
maintenant dans la tâche readPotentiometer(). Pour avoir accès au données en temps réel, la fonction
utilise xQueueReceive() qui permet de réatitrer les valeurs les plus récents aux éléments de la structs,
mais seulement si un changement a été perçu. Sinon, les anciennes valeurs sont utilisés. Cette utilisation
permet de diminuer les calculs et facilement accéder aux valeurs nécéssaire pour une fonction qui est
appelé à une haute fréquence tel que celle-ci
*/
int8_t processVCF(int8_t input)
{
    static int8_t y1 = 0;
    static int8_t y2 = 0;

    if (xQueueReceive(filterCoeffsQueue, &sFilterCoeffs, 0))
    {
    }

    float b1 = sFilterCoeffs.b1;
    float a1 = sFilterCoeffs.a1;
    float a2 = sFilterCoeffs.a2;

    int16_t y0 = b1 * input + a1 * y1 + a2 * y2;

    int8_t output = 0xFF & (y0 >> 8);

    y2 = y1;
    y1 = output;

    return output;
}

/*
La fonction processVCA permet de calculer la chute linéaire de la fréquence joué. Pour ce faire, la fonction
accède premièrement aux valeurs les plus récentes des structs pertinentes. La fonction calcule premièrement
le nombre d'itérations nécéssaire pour compléter la chute selon le temps déterminer par la valeur du
potentiomètre. Si l'un des deux boutons est enclenché, le nombre d'itérations est contournés et la valeur
de fréquence est déterminé par le paramètre d'entré de la fonction (le vcf). Si l'un des deux boutons n'est
plus enclenché, la chute débute. Une fois le nombre d'itérations complété, la fréquence de sortie est posé
à 0 afin de marquer la fin de la chute.
*/
int8_t processVCA(int8_t input)
{
    if (xQueueReceive(potDataQueue, &sPotentiometerData, 0))
    {
    }

    if (xQueueReceive(switchDataQueue, &sSwitchData, 0))
    {
    }

    float bufferIterations = sPotentiometerData.vcaLength * 8000;
    static float iteration = bufferIterations;
    int8_t frequencyOutput;

    if (sSwitchData.sw1 || sSwitchData.sw2)
    {
        iteration = 0;
        frequencyOutput = input;
    }
    else
    {
        if (iteration < bufferIterations)
        {
            float decay = 1.0f - (iteration / bufferIterations);
            frequencyOutput = input * decay;
            iteration++;
        }
        else
        {
            frequencyOutput = 0;
        }
    }

    return frequencyOutput;
}

/*
L fonction nextSample() a grandement été modifié pour gérer le cas de la mélodie. Si le bouton sw1 est
enclenché, le comportement reste le même et la même note est envoyé au vco. Par contre, si le bouton sw2 est
enclenché, chacune des notes doivent maintenant etre joué correctement. Pour ce faire, la fonction détermine
d'abord le temps depuis le début du programme. Un calcul permet ensuite de déterminer si la duration de la
note est respecté. La fonction permet ensuite de gérer le cas où la chanson doit jouer en boucle et finalement
donné la valeur de la fréquence de la note a la fonction setNoteHz() qui est ensuite utilisé dans le vco. Les
queues sont encore utilisé dans cette fonction afin de faciliter l'accès à des variables dans plusieurs
fonctions tout en les gardant "thread safe"
*/
int8_t nextSample()
{
    int8_t vco;

    if (xQueueReceive(potDataQueue, &sPotentiometerData, 0))
    {
    }

    if (sSwitchData.sw1)
    {
        vco = sawtooth_.next() + squarewv_.next();
    }
    else if (sSwitchData.sw2)
    {
        unsigned long currentTime = millis();

        // TODO Implement tempo division on calculation
        if (currentTime - sMelodyData.noteStartTime >= (song[sMelodyData.currentNoteIndex].duration * TEMPO_16T_MS))
        {
            sMelodyData.currentNoteIndex++;

            if (sMelodyData.currentNoteIndex >= sizeof(song) / sizeof(song[0]))
            {
                sMelodyData.currentNoteIndex = 0;
            }

            setNoteHz(song[sMelodyData.currentNoteIndex].freq);
            sMelodyData.noteStartTime = currentTime;
        }

        vco = sawtooth_.next() + squarewv_.next();
    }
    else
    {
        vco = sawtooth_.next() + squarewv_.next();
    }

    int8_t vcf = processVCF(vco);
    int8_t vca = processVCA(vcf);

    return vca;
}

/*
Les 2 fonctions de ISR des boutons poussoirs servent le meme but. Les 2 sont enclenché au moments de leurs
interrupts respectifs. En premier lieu, les fonction déclare une variable de type BaseType_t. Cette variable
permet de signaler au système si une tâche de plus haute priorité est réveillé lors de l'appel de la
fonction xQueueOverwriteFromISR(). Cette fonction est faite pour etre executé dans un ISR et est dont tres
rapide et efficace. Son utilité est de mettre a jour l'élément de la struct des boutons poussoirs et
d'esnsuite mettre a joure la queue
*/
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

/*
La lecture des potentiomètres n'est effectués qu'au 10Hz. C'est donc un excellent moment pour effectuer
des calculs qui sont souteux, car la fonction n'est pas effectué souvent. Ainsi, une partie des calculs
original de la fonction nextSample() ont été migré ici. Chacune des valeurs de potentiomètre est
correctement mapper sur les valeurs déterminés et sont ensuite attribuer à un des élément de la struct
contenant les différents attributs des potentiomètres. De plus, une fois les calculs de b1, a1 et a2
effectué leur résultat est également stocké dans leur struct respective. Une fois ceci fait, les queues
sont ensuite mises à jour avec les résultats déterminés
*/
void readPotentiometer(void *pvParameters __attribute__((unused)))
{
    for (EVER)
    {
        sPotentiometerData.tempo = analogRead(PIN_RV1) * 180.0f / 1023.0f + 60.0f;
        sPotentiometerData.vcaLength = (analogRead(PIN_RV2) / 1023.0f) * 3.0f;
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

/*
La fonction fillBuffer ne fait que remplir le buffer avec la fréquence pré déterminé. Si le buffer est
plein, la tâche est relégué
*/
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

    Serial.begin(9600);

    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);

    /*
    Pour l'implémentation des boutons, il a été déterminé que l'utilisation des interrupt de FreeRTOS
    permettrais une implémentation plus dynamique. En effet, On ne vérifie pas en permannance si les
    boutons sont enclenché, mais dès qu'un changement d'état est déclanché, celui-ci notifie le ISR
    respectif
    */

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), sw1Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), sw2Isr, CHANGE);

    /*
    Une Queue est créée pour chacune des struct définies plus haut
    */
    switchDataQueue = xQueueCreate(1, sizeof(SwitchData));
    potDataQueue = xQueueCreate(1, sizeof(PotentiometerData));
    filterCoeffsQueue = xQueueCreate(1, sizeof(FilterCoeffs));
    melodyDataQueue = xQueueCreate(1, sizeof(MelodyData));

    setNoteHz(440.0);

    pcmSetup();

    /*
    Seulement 2 taches sont utilisés. La lecture des potentiomètres sont priorisé pour permettre un
    changement plus dynamique. Le buffer quant à lui est une tâche secondaire qui est quand mème
    exécuté la majorité du temps puisque les potentiomètres ne sont lue qu'à une fréquence de 10Hz
    */
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