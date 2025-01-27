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

static constexpr float baseTime = 60.0f * 250.0f;
int song_lenght = sizeof(song) / sizeof(song[0]);

/*
Les structs suivantes sont définies afin de limiter la création et la gestion de queues dans le programme.
De cette manière, il est possible d'éviter le overhead non nécéssaire. Les structs sont séparés selon des
groupes déterminés. Toutes les valeurs auraient pu être mises dans une seule queues, mais cela nuirait à
la lisibilité ainsi qu'à la modularité. Une seule des tâches (readPotentiometer()) crée des objets locales.
De cette manière les objets ne sont créer que lors de la lecture et la modification des valeurs. Cette 
manière de faire permet également de diminuer la gestions des queues et la création des structs. 
Les queues ont été utilisés pour permettre l'accès aux valeurs de ces structs dans plusieurs fonctions de 
façon "thread safe".Un event group aurait pu être utilisé pour les boutons, mais comme nous utilisions des 
queues pour les autres valeurs, nous avons décidé de rester cohérent. Les queues de lectures sont globales, 
de cette façon il y a moins d'overhead. De plus, elles ne peuvent pas se nuire entre elles, car elles sont 
utilisées dans la même tâche.
*/

struct SwitchData
{
    bool sw1 = false;
    bool sw2 = false;
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
    int currentNoteDuration = 0;
} sMelodyData;

QueueHandle_t switchDataQueue;
QueueHandle_t potDataQueue;
QueueHandle_t filterCoeffsQueue;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

/*
La fonction processVCF demeure presque identique à l'exeption qu'une partie des calculs se trouvent
maintenant dans la tâche readPotentiometer(). Les valeurs accéder dans cette fonctions sont 
considérés comme globales. Ce choix peut être fait de par l'utilisation de ces variables dans une 
seule tâche. Étant donné que processVCF est une fonction utilisé dans cette tâche, ceci ne pose pas
problème.
*/
int8_t processVCF(int8_t input)
{
    static int8_t y1 = 0;
    static int8_t y2 = 0;

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
accède aux données de la même manière que la fonction processVCF. La fonction calcule premièrement
le nombre d'itérations nécéssaire pour compléter la chute selon le temps déterminer par la valeur du
potentiomètre. Si l'un des deux boutons est enclenché, le nombre d'itérations est contournés et la valeur
de fréquence est déterminé par le paramètre d'entré de la fonction (le vcf). Si l'un des deux boutons n'est
plus enclenché, la chute débute. Une fois le nombre d'itérations complété, la fréquence de sortie est posé
à 0 afin de marquer la fin de la chute.
*/
int8_t processVCA(int8_t input)
{
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
La fonction nextSample() a grandement été modifié pour gérer le cas de la mélodie. Si le bouton sw1 est
enclenché, le comportement reste le même et la même note est envoyé au vco. Par contre, si le bouton sw2 est
enclenché, chacune des notes doivent maintenant etre joué correctement. Pour ce faire, la fonction détermine
le nombre de case de buffer à remplir selon le potentiomètre du tempo. La fonction permet ensuite de gérer le 
cas où la chanson doit jouer en boucle et finalement donné la valeur de la fréquence de la note a la fonction 
setNoteHz() qui est ensuite utilisé dans le vco. Dans cette fonction, les valeurs des queues sont réassignés
grâce à xQeuePeek(). Cela permet de lire la dernière valeur obtenu selon la lecture des capteurs tout en restant
thread safe.
*/
int8_t nextSample()
{
    int8_t vco;

    xQueuePeek(filterCoeffsQueue, &sFilterCoeffs, 0);
    xQueuePeek(potDataQueue, &sPotentiometerData, 0);
    xQueuePeek(switchDataQueue, &sSwitchData, 0);

    if (sSwitchData.sw1)
    {
        vco = sawtooth_.next() + squarewv_.next();
    }
    else if (sSwitchData.sw2)
    {
        if (sMelodyData.currentNoteIndex < song_lenght)
        {   
            if (sMelodyData.currentNoteDuration < song[sMelodyData.currentNoteIndex].duration * baseTime / sPotentiometerData.tempo)
            
            {
                setNoteHz(song[sMelodyData.currentNoteIndex].freq);
                sMelodyData.currentNoteDuration++;
            }
            else
            {
                sMelodyData.currentNoteIndex++;
                sMelodyData.currentNoteDuration = 0;
            }
        }
        else
        {
            sMelodyData.currentNoteIndex = 0;
            sMelodyData.currentNoteDuration = 1;
            setNoteHz(song[sMelodyData.currentNoteIndex].freq);
        }
        vco = sawtooth_.next() + squarewv_.next();
    }
    else
    {
        sMelodyData.currentNoteIndex = 0;
        sMelodyData.currentNoteDuration = 0;
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
d'esnsuite mettre a joure la queue. Les isr ont été utilisé, car les boutons ont un comportement apériodique.
*/
void sw1Isr()
{
    SwitchData sSwitchDataLocal;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    sSwitchDataLocal.sw1 = digitalRead(PIN_SW1);

    xQueueOverwriteFromISR(switchDataQueue, &sSwitchDataLocal, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void sw2Isr()
{
    SwitchData sSwitchDataLocal;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    sSwitchDataLocal.sw2 = digitalRead(PIN_SW2);

    xQueueOverwriteFromISR(switchDataQueue, &sSwitchDataLocal, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/*
La lecture des potentiomètres n'est effectués qu'au 10Hz. C'est donc un excellent moment pour effectuer
des calculs qui sont couteux, car la fonction n'est pas effectué souvent. Ainsi, une partie des calculs
original de la fonction nextSample() ont été migré ici. Chacune des valeurs de potentiomètre est
correctement mapper sur les valeurs déterminés et sont ensuite attribuer à un des élément de la struct
contenant les différents attributs des potentiomètres. De plus, une fois les calculs de b1, a1 et a2
effectué leur résultat est également stocké dans leur struct respective. Une fois ceci fait, les queues
sont ensuite mises à jour avec les résultats déterminés.
*/
void readPotentiometer(void *pvParameters __attribute__((unused)))
{
    PotentiometerData sPotentiometerDataLocal;
    FilterCoeffs sFilterCoeffsLocal;

    TickType_t taskWakeTime;
    const TickType_t taskFrequency= 100;

    taskWakeTime = xTaskGetTickCount();

    for (EVER)
    {
        sPotentiometerDataLocal.tempo = analogRead(PIN_RV1) * 180.0f / 1023.0f + 60.0f;
        sPotentiometerDataLocal.vcaLength = (analogRead(PIN_RV2) / 1023.0f) * 3.0f;
        sPotentiometerDataLocal.coupure = (analogRead(PIN_RV3) * PI) / 1023.0f;
        sPotentiometerDataLocal.frequence = analogRead(PIN_RV4) / 1023.0f;

        float f = sPotentiometerDataLocal.coupure;
        float q = sPotentiometerDataLocal.frequence;

        float fb = (q + (q / (1.0f - f)));
        sFilterCoeffsLocal.b1 = f * f * 256.0f;
        sFilterCoeffsLocal.a1 = (2 - 2 * f + f * fb - f * f * fb) * 256.0f;
        sFilterCoeffsLocal.a2 = -(1 - 2 * f + f * fb + f * f - f * f * fb) * 256.0f;

        xQueueOverwrite(potDataQueue, &sPotentiometerDataLocal);
        xQueueOverwrite(filterCoeffsQueue, &sFilterCoeffsLocal);

        vTaskDelayUntil(&taskWakeTime, taskFrequency);
    }
}

/*
La fonction fillBuffer ne fait que remplir le buffer avec la fréquence pré déterminé. Si le buffer est
plein, la tâche est relégué. Dans notre cas, il n'y a pas d'autres tâches inférieur. Cependant, d'un point de vue d'implémentation future 
et de modularité. Le taskYIELD() est quand même présent.
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
    sawtooth_ = Sawtooth(SAW2048_DATA);

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
        512,
        NULL,
        3,
        NULL);
}

void loop()
{
}

/*
Notre code possède certaines limites. En effet,  
nous avons une très grand précision sur la mélodie et le nombre de cycle à jouer pour l'atténuation du VCA.
Cependant, il aurait été possible de faire tout avec des vTaskDelay. La programmation aurait été plus simple, mais 
nous aurions eu moins de contrôle sur les notes jouées. En effet, nous avons dû faire beaucoup d'optimisation afin de réduire
le jeeter au minimum. Effectivement, beaucoup de calcul sont introduits dans chaque ajout d'une note au buffer ce qui crée du délais 
dans l'ajout des notes
Bref, nous avons choisis la précision et le contrôle sur la mélodie, mais celle-ci viens avec un coup de cpu plus élevé. 
*/