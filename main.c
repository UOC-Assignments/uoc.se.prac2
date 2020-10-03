/******************************************************************************

                        Sistemes Encastats - UOC 2019/20-1

                               PR�CTICA - Projecte 2

                                 Jordi Bericat Ruz

*******************************************************************************/


/*------------------------------------------------------------------------------

                                      INCLUDES

------------------------------------------------------------------------------*/


/* Standard includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"

/* Boosterpack Accelerometer drivers includes */
#include "edu_boosterpack_accelerometer.h"

/* Boosterpack RGB drivers includes */
#include "edu_boosterpack_rgb.h"

/* LCD related drivers includes */
#include "st7735.h"
#include "st7735_msp432.h"
#include "grlib.h"

/* UART drivers */
#include "uart_driver.h"


/*------------------------------------------------------------------------------

                                      CONSTANTS

------------------------------------------------------------------------------*/

/* Prioritats de cada tasca */
#define UART_TASK_PRIORITY          ( tskIDLE_PRIORITY + 5 )
#define ADC_TASK_PRIORITY           ( tskIDLE_PRIORITY + 4 )
#define PROCESSING_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define LCD_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

/* Mida de l'stack assignat a cada tasca del total del heap */
#define UART_TASK_STACK_SIZE        ( 1024 )
#define ADC_TASK_STACK_SIZE         ( 1024 )
#define PROCESSING_TASK_STACK_SIZE  ( 1024 )
#define LCD_TASK_STACK_SIZE         ( 1024 )
#define HEARTBEAT_TASK_STACK_SIZE   ( 128 )

/* Mida de les cues */
#define QUEUE_SIZE                  ( 15 )

/* Retards (vTaskDelay) */
#define HEART_BEAT_ON_MS            ( pdMS_TO_TICKS(10) )
#define HEART_BEAT_OFF_MS           ( pdMS_TO_TICKS(990) )
#define PROCESSING_TASK_DELAY_MS    ( pdMS_TO_TICKS(500) )
#define ADC_TASK_DELAY_MS           ( pdMS_TO_TICKS(10) )
#define COMMAND_DELAY_MS            ( pdMS_TO_TICKS(2000) )
#define START_DELAY_MS              ( pdMS_TO_TICKS(1000) )

/* Valors promig de l'acceler�metre */
#define ZERO_G_ACC_X                ( 8300.00 )
#define ZERO_G_ACC_Y                ( 8260.00 )
#define ZERO_G_ACC_Z                ( 8300.00 )
#define ONE_G_ACC_X                 ( 5130.00 )
#define ONE_G_ACC_Y                 ( 4930.00 )
#define ONE_G_ACC_Z                 ( 4900.00 )

/* Els seg�ents factors s'utilitzen per a fer la conversi� lineal de l'angle existent entre els plans xz i yz a
 * les coordenada x i y del display LCD respectivament segons l'equaci� de la recta f(x) = m*x + b (M_FACTOR i
 * B_FACTOR s'han obtingut calculant el pendent de la gr�fica x/f(x) tenint en compte que f(-90) = 0, f(-45) = 32,
 * f(0) = 64, f(45) = 96 i f(90) = 128 ) */
#define M_FACTOR                    ( 0.711 )
#define B_FACTOR                    ( 64.005 )
#define RGB_FACTOR                  ( 5.6667 )

/* Constants auxiliars */
#define ERROR_THRESHOLD             ( 2.00 )
#define MEAN_VALUES                 ( 10.00 )
#define PI                          ( 3.14159265 )
#define PI_RAD_DEG                  ( 180.00 )
#define COMMAND_ERROR               ( 99 )


/*------------------------------------------------------------------------------

                                 TIPUS PREDEFINITS

------------------------------------------------------------------------------*/


/* Estructura utilitzada per a encapsular la comanda rebuda per UART i desar-la a la variable global "command" */
typedef struct {
    int8_t x;
    int8_t y;
} command_t;

/* Estructura utilitzada per a encapsular els valors de lectura de l'acceler�metre i enviar-los desde la tasca
 * "ADCTask" a la tasca "ProcessingTask" mitjan�ant la cua "xQueueADC2Process"*/
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} acc_t;

/* Estructura utilitzada per a encapsular els resultats dels c�lculs realitzats a la tasca "ProcessingTask" i
 * enviar-los a la tasca "LCDTask" mitjan�ant la cua "xQueueProcess2LCD" */
typedef struct {
    float xz;
    float yz;
    TickType_t TotalTicks;
} LCD_result_t;


/*------------------------------------------------------------------------------

                               PROTOTIPS DE FUNCIONS

------------------------------------------------------------------------------*/


/* Tasques */
static void UARTReadingTask(void *pvParameters);
static void ADCTask(void *pvParameters);
static void ProcessingTask(void *pvParameters);
static void HeartBeatTask(void *pvParameters);
static void LCDTask(void *pvParameters);

/* ISR Callbacks */
void uart_rx_callback(circ_buffer_t*);
void get_adc_results(adc_result);

/* Funcions auxiliars */
command_t uart_to_command(void);


/*------------------------------------------------------------------------------

                                 OBJECTES GLOBALS

------------------------------------------------------------------------------*/


/*** Mecanismes de sincronitzaci� de tasques ***/

/*Sem�for utilitzat per a disparar la tasca "UARTReadingTask" cada cop que es generi una interrupci� eUSCI (UART) */
SemaphoreHandle_t xRunUARTReadingTaskSemaphore;
SemaphoreHandle_t xRunADCTaskSemaphore;
SemaphoreHandle_t xADCReadingAvailableSemaphore;
QueueHandle_t xQueueADC2Process;
QueueHandle_t xQueueProcess2LCD;

/*** Variables Globals ***/

/* Configuraci� / inicialitzaci� del display LCD */
static Graphics_Context g_sContext;

/* Timestamp d'inici de lectures de l'acceler�metre */
TickType_t start_timestamp;

/* Lectures de l'acceler�metre / ADC */
acc_t accel_values;

/* Aquesta variable la utilitzem per a desar les dades obtingudes del buffer del UART desde la funci� "uart_rx_callback"
 * (invocada desde la ISR "EUSCIA0_IRQHandler") i poder llegir-les desde la tasca "prvUARTReadTask" */
char uart_command[10];

/* La comanda rebuda per UART s'assignara a aquesta variable, la qual ser� enviada per la cua a la tasca "ProcessingTask" */
command_t command;

/* Aquesta variable s'utilitzar� per a comparar la posici� de la "bola" que s'imprimeix al display LCD amb la posici� de
 * la mateixa al frame anterior, de manera que si les coordenades s�n les mateixes no es tornar� a imprimir la bola
 * (aquest mecanisme permet evitar l'efecte de "pampallugueix" de la bola quan no hi ha moviment de la placa) */
uint8_t previous_lcd_x;
uint8_t previous_lcd_y;


/*------------------------------------------------------------------------------

                                       MAIN

------------------------------------------------------------------------------*/


int main(int argc, char** argv)
{
    int32_t retVal = -1;

    /* Inicialitzaci� de sem�fors, m�texs i cues */
    xRunUARTReadingTaskSemaphore = xSemaphoreCreateBinary ();
    xRunADCTaskSemaphore =  xSemaphoreCreateBinary ();
    xADCReadingAvailableSemaphore =  xSemaphoreCreateBinary ();
    xQueueADC2Process = xQueueCreate( QUEUE_SIZE, sizeof( acc_t ) );
    xQueueProcess2LCD = xQueueCreate( QUEUE_SIZE, sizeof( LCD_result_t ) );

    /* Inicialitzaci� de la placa */
    board_init();

    /* Inicialitzaci� de l'acceler�metre i establiment de la funci� de callback que s'executar� cada cop que es
     * generi una interrupci� al modificar-se dels registres de mem�ria de l'ADC */
    edu_boosterpack_accelerometer_init();
    edu_boosterpack_accelerometer_set_callback(get_adc_results);

    /* Inicialitzaci� del led RGB */
    edu_boosterpack_rgb_init();

    /* Inicialitzaci� i configuraci� del display LCD */
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);

    /* Inicialitzaci� de la UART (indiquem com a par�metre d'entrada la funci� callback que s'executar� cada cop que
     * es premi la tecla intro al terminal serie). Cada cop que es cridi la funci� callback s'alliberar� el sem�for
     * binari que dispara la tasca "prvUARTReadTask" */
    uart_init(uart_rx_callback);

    /* Enviem missatge de benvinguda pel port s�rie */
    uart_print("\n\r\n\rIMPLEMENTACIO PRACTICA - PROJECTE 2 - Sistemes Encastats - UOC\n\r\n\rIntrodueix la comanda en format (X,Y) --> ");

    if (       (xRunUARTReadingTaskSemaphore != NULL)
            && (xADCReadingAvailableSemaphore != NULL)
            && (xRunADCTaskSemaphore != NULL)
            && (xQueueADC2Process != NULL)
            && (xQueueProcess2LCD != NULL) ) {

        /* Creaci� de la tasca "UARTReadingTask" */
        retVal = xTaskCreate(UARTReadingTask,
                             "UARTReadingTask",
                             UART_TASK_STACK_SIZE,
                             NULL,
                             UART_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "ADCTask" */
        retVal = xTaskCreate(ADCTask,
                             "ADCTask",
                             ADC_TASK_STACK_SIZE,
                             NULL,
                             ADC_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "ProcessingTask" */
        retVal = xTaskCreate(ProcessingTask,
                             "ProcessingTask",
                             PROCESSING_TASK_STACK_SIZE,
                             NULL,
                             PROCESSING_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "LCDTask" */
        retVal = xTaskCreate(LCDTask,
                             "LCDTask",
                             LCD_TASK_STACK_SIZE,
                             NULL,
                             LCD_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "HeartBeatTask" */
        retVal = xTaskCreate(HeartBeatTask,
                             "HeartBeatTask",
                             HEARTBEAT_TASK_STACK_SIZE,
                             NULL,
                             HEARTBEAT_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Inicialitzaci� de l'scheduler */
        vTaskStartScheduler();
    }
    return 0;
}


/*------------------------------------------------------------------------------

                                        TASQUES

------------------------------------------------------------------------------*/


static void UARTReadingTask(void *pvParameters)
{

    /* Temps m�xim d'espera entre dues interrupcions rx del port UART */
    const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS(100);

    char tmp_str[12];

    // La tasca es repeteix en un bucle infinit
    for (;;)
    {
        /* El sem�for s'entregar� per la ISR EUSCIA0_IRQHandler (espera un nombre m�xim de "xMaxExpectedBlockTime" ticks) */
        if (xSemaphoreTake(xRunUARTReadingTaskSemaphore, xMaxExpectedBlockTime) == pdPASS)
        {

            /* Neteixem el display LCD abans de comen�ar el proc�s d'anivellat */
            Graphics_clearDisplay(&g_sContext);

            /* Amb la funci� "uart_to_command" Fem el "parsing" de les dades obtingudes per la UART a la variable
             * string global "uart_string" cap a la variable local de tipus color_t "command_to_send", que s'enviar� per
             * la cua a la tasca "RGBTask". */
            command = uart_to_command();

            /* Si la comanda est� mal formatada, la variable command.x contindr� el valor "99". En aquest cas imprimirem la
             * cadena "Comanda Incorrecta" al display LCD... */
            if (command.x == COMMAND_ERROR)
            {
                Graphics_drawStringCentered(&g_sContext,
                                    "Comanda Incorrecta",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    64,
                                    OPAQUE_TEXT);
            }

            /* ...Si no, imprimim la comanda al display LCD i iniciem el proc�s d'anivellament de la placa */
            else
            {
                sprintf(tmp_str,"Nivell %d,%d",command.x,command.y);

                Graphics_drawStringCentered(&g_sContext,
                                    "Comanda correcta:",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    45,
                                    OPAQUE_TEXT);

                Graphics_drawStringCentered(&g_sContext,
                                    (int8_t*)tmp_str,
                                    AUTO_STRING_LENGTH,
                                    64,
                                    70,
                                    OPAQUE_TEXT);

                /* Imposem un retard de 2 segons abans de mostrar la cadena "start" al display LCD */
                vTaskDelay(COMMAND_DELAY_MS);

                /* Esborrem tot el contingut del LCD i mostrem la cadena "start" */
                Graphics_clearDisplay(&g_sContext);
                Graphics_drawStringCentered(&g_sContext,
                                    "START",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    64,
                                    OPAQUE_TEXT);

                /* Imposem un retard d'un segon despr�s de mostrar la cadena "start" al display LCD */
                vTaskDelay(START_DELAY_MS);

                /* Esborrem el contingut del display */
                Graphics_clearDisplay(&g_sContext);

                /* Desem el "timestamp" inicial a una variable global per tal de poder calcular el temps total que s'ha
                 *  trigat en anivellar la placa */
                start_timestamp = xTaskGetTickCount();

                /* Alliberem el sem�for que permet la execuci� de la tasca "ADCTask" */
                xSemaphoreGive(xRunADCTaskSemaphore);
            }

            /* Mostrem al terminal s�rie el "prompt" per a l'interpret de comandes */
            uart_print("Introdueix la comanda en format (X,Y) --> ");
        }
    }
}


static void ADCTask(void *pvParameters){

    for(;;){

        if( xSemaphoreTake( xRunADCTaskSemaphore, portMAX_DELAY ) == pdTRUE ){

            /* Obtenim la lectura actual de l'acceler�metre. Concretament, La seg�ent crida permetra desar als registres de
             * mem�ria de l'ADC les lectures de les entrades anal�giques configurades per llegir els 3 canals de sortida
             * (x, y i z) de l'acceler�metre. La ISR de l'ADC utilitzar� la funci� de callback "get_adc_results" per a retornar
             * les lectures, les quals ser�n desades a la variable global "accel_values" per a ser finalment enviades per la cua
             * a la tasca "ProcessingTask". */
            edu_boosterpack_accelerometer_read();

            /* Enviem la lectura de l'ADC a la tasca "ProcessingTask" mitjan�ant la cua "xQueueADC2Process" per tal de realitzar
             * els c�lculs corresponents */
            if( xSemaphoreTake( xADCReadingAvailableSemaphore, portMAX_DELAY ) == pdTRUE ){
                xQueueSend(xQueueADC2Process, &accel_values, portMAX_DELAY);
            }
        }
    }
}

static void ProcessingTask(void *pvParameters) {

    /* Declarem i inicialitzem variables locals */
    float mean_acc_x = 0;
    float mean_acc_y = 0;
    float mean_acc_z = 0;

    float g_value_x = 0;
    float g_value_y = 0;
    float g_value_z = 0;

    float xz_angle = 0;
    float yz_angle = 0;

    float xz_error = 0;
    float yz_error = 0;

    uint8_t red_rgb_led;
    uint8_t blue_rgb_led;

    acc_t new_accel_value;

    LCD_result_t result;
    result.TotalTicks = 0;

    for (;;)
    {

        /* La tasca es desbloqueja un cop hi ha dades a la cua (mostres de l'ADC) i la tasca de prioritat superior es troba en estat
         * de bloqueig (sem�for "xRunADCTaskSemaphore" reservat per� no alliberat) */
        if ( xQueueReceive(xQueueADC2Process, &new_accel_value, portMAX_DELAY) == pdPASS)
        {

            /* Calculem els valors promig de les lectures de l'acceler�metre */
            mean_acc_x = (float) new_accel_value.x * (1 / MEAN_VALUES) + mean_acc_x * ((MEAN_VALUES - 1) / MEAN_VALUES);
            mean_acc_y = (float) new_accel_value.y * (1 / MEAN_VALUES) + mean_acc_y * ((MEAN_VALUES - 1) / MEAN_VALUES);
            mean_acc_z = (float) new_accel_value.z * (1 / MEAN_VALUES) + mean_acc_z * ((MEAN_VALUES - 1) / MEAN_VALUES);

            /* Calculem cada component del vector G (Gx, Gy, Gz) en funci� del valor actual promig de cada eix de l'acceler�metre */
            g_value_x = ((mean_acc_x - ZERO_G_ACC_X) / ONE_G_ACC_X);
            g_value_y = ((mean_acc_y - ZERO_G_ACC_Y) / ONE_G_ACC_Y);
            g_value_z = ((mean_acc_z - ZERO_G_ACC_Z) / ONE_G_ACC_Z);

            /* Si la for�a G efectuada sobre l'eix Z �s igual zero (a la pr�ctica, < 0.1 aproximadament), aleshores hem de calcular
             * l'angle de X en funci� de l'eix Y (pla XY). Despr�s tindrem en compte que la funci� atan() ens retorna l'angle en
             * radians, fet pel qual hem de fer la conversi� a graus multiplicant per 180 graus (PI_RAD_DEG) i dividint per PI */
            if (g_value_z > -0.1 && g_value_z < 0.1)
            {
                xz_angle = atan(g_value_x / g_value_y) * PI_RAD_DEG / PI;
                yz_angle = atan(g_value_y / g_value_x) * PI_RAD_DEG / PI;
            }
            else
            {
                xz_angle = atan(g_value_x / g_value_z) * PI_RAD_DEG / PI;
                yz_angle = atan(g_value_y / g_value_z) * PI_RAD_DEG / PI;
            }

            /* Calculem el diferencial d'error entre la inclinaci� indicada a la comanda rebuda per UART i la inclinaci� actual */
            xz_error = command.x - xz_angle;
            yz_error = command.y - yz_angle;

            /* Si xz_error & yz_error s�n m�s petits o iguals a 2 graus, aleshores tenim la placa anivellada */
            if (abs(xz_error) <= ERROR_THRESHOLD && abs(yz_error) <= ERROR_THRESHOLD)
            {

                /* calculem la quantitat de ticks totals que han passat des de que s'ha comen�at a prendre mostres del ADC fins que
                 * s'ha anivellat la placa. Li sumem 1ms a mode de "workaround" ja que en els casos en qu� la placa es troba anivellada
                 * d'inici ens trobem amb qu� totalTicks = 0 i en conseq��ncia la tasca LCDTask no pot discriminar per TotalTime per tal
                 * de seguir prenent mesures de l'ADC o mostrar el missatge de placa anivellada (es tracta d'una limitaci� de la implementaci�
                 * escollida i ja no em queda temps per a pensar-ne una altra) */
                result.TotalTicks = xTaskGetTickCount() - start_timestamp + 1;

                /* Apaguem leds RGB i vermell i encenem el verd */
                edu_boosterpack_rgb_pwm_all(0, 0, 0);
                led_off(MSP432_LAUNCHPAD_LED_RED1);
                led_on(MSP432_LAUNCHPAD_LED_GREEN);

                /* Imposem un retard de 500ms per tal de poder observar que el led verd s'enc�n */
                vTaskDelay(PROCESSING_TASK_DELAY_MS);

                /* Apaguem el led verd */
                led_off(MSP432_LAUNCHPAD_LED_GREEN);
            }

            /* Si no, aleshores obtenim els valors d'intensitat de cada canal del led RGB en funci� de la lectura de l'acceler�metre
             * i seguidament enviem el diferencial d'inclinaci� de placa respecte de l'angle desitjat a la tasca "LCDTask" per tal de mostrar
             * la "bola" que hem d'imprimir al display a la coordenada corresponent */
            else
            {

                /* Si l'angle actual difereix en +-45�, el led RGB s'il�lumina amb intensitat m�xima, si no, aleshores calculem la itensitat
                 * que ha de tenir el canal corresponent del led RGB */
                if (xz_error > -45 && xz_error < 45)
                {
                    red_rgb_led = (uint8_t)( RGB_FACTOR * abs(xz_error) );
                }
                else
                {
                    red_rgb_led = 255;
                }
                if (yz_error > -45 && yz_error < 45)
                {
                    blue_rgb_led = (uint8_t)( RGB_FACTOR * abs(yz_error) );
                }
                else
                {
                    blue_rgb_led = 255;
                }

                /* Activem el led RGB amb els valors d'intensitat obtinguts per a cada canal */
                edu_boosterpack_rgb_pwm_all( red_rgb_led, 0, blue_rgb_led );

                /* Encapsulem els c�lculs per tal d'enviar-los a la tasca LCDTask */
                result.TotalTicks = 0;
                result.xz = xz_error;
                result.yz = yz_error;
            }

            /* Enviem el diferencial de timestamps i dels angles xz i yz a la tasca LCDTask amb la cua corresponent, que utilitzem
             * per a la comunicaci� i sincronitzaci� inter-tasca */
            xQueueSend(xQueueProcess2LCD, &result, portMAX_DELAY);
        }
    }
}

static void LCDTask(void *pvParameters){

    LCD_result_t result;
    unsigned long total_ms;
    char total_ms_str[10];

    /* S'ha hagut d'utilitzar el tipus "float" per a poder realitzar el c�lcul de cada coordenada (despr�s es fara un cast a uint_8)  */
    float lcd_x, lcd_y;

    for(;;){

        /* Rebem diferencial de timestamps de la cua */
        if( xQueueReceive( xQueueProcess2LCD, &result, portMAX_DELAY ) == pdPASS )
        {
            if(result.TotalTicks == 0){

                /* Imprimim el cercle central al display LCD que ens serveix de guia per a anivellar la placa */
                Graphics_drawCircle(&g_sContext, 64, 64, 10);

                /* calculem la coordenada X i Y on s'ha d'imprimir la "bola" al display LCD (veure comentari a la l�nia 88 per a
                 * informaci� ampliada) */
                lcd_x = result.xz*M_FACTOR+B_FACTOR;
                lcd_y = result.yz*M_FACTOR+B_FACTOR;

                /* Esborrem el cercle (bola) del frame anterior per a qu� no s'encavalqui amb el que tot seguit es tornar� a generar
                 * (en cas de no aconseguir l'anivellat) un cop es torni a realitzar la seg�ent lectura del ADC */
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_fillCircle(&g_sContext, previous_lcd_x, previous_lcd_y, 5);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

                /* Imprimim "la bola" que ens indica quina �s la inclinaci� actual de la placa al respecte de la inclinaci� desitjada
                 * especificada a la comanda per UART */
                Graphics_fillCircle(&g_sContext, (uint8_t)lcd_x, (uint8_t)lcd_y, 5);

                /* Desem les coordenades actuals de la bola a les seg�ents variables globals per tal de poder esborrar-la al seg�ent
                 * cicle d'execuci� d'aquesta tasca */
                previous_lcd_x = (uint8_t)lcd_x;
                previous_lcd_y = (uint8_t)lcd_y;

                /* Si la placa encara no est� anivellada en aquest punt hem de seguir realitzant lectures
                 * de l'ADC cada 10 ms, fet pel qual imposem un retard. */
                vTaskDelay(ADC_TASK_DELAY_MS);

                /* Finalment alliberem el sem�for que permet el desbloqueig de la tasca ADCTAsk de manera que podrem seguir prenent
                 * mostres de l'acceler�metre */
                xSemaphoreGive( xRunADCTaskSemaphore );

            } else {
                /* Convertim els ticks a ms (multipliquem * 10 ja que la resoluci� tick est� configurada de manera que "1ms < 1 tick < 10ms")... */
                total_ms = ( result.TotalTicks / pdMS_TO_TICKS(10) )*10;

                /*... I els desem en una variable de tipus string per a poder imprimir al display LCD */
                sprintf(total_ms_str,"%lu ms",total_ms);

                /* Neteixem el display */
                Graphics_clearDisplay(&g_sContext);

                /* Imprimim per pantalla el missatge final i la quanitat de ms que han passat des de l'inici */
                Graphics_drawStringCentered(&g_sContext,
                                    "Nivell aconseguit!",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    30,
                                    OPAQUE_TEXT);

                Graphics_drawStringCentered(&g_sContext,
                                    "Temps transcorregut:",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    60,
                                    OPAQUE_TEXT);

                Graphics_drawStringCentered(&g_sContext,
                                    (int8_t*)total_ms_str,
                                    AUTO_STRING_LENGTH,
                                    64,
                                    90,
                                    OPAQUE_TEXT);
            }
        }
    }
}

static void HeartBeatTask(void *pvParameters){

    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( HEART_BEAT_ON_MS );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( HEART_BEAT_OFF_MS );
    }
}


/*------------------------------------------------------------------------------

                                   ISR CALLBACKS

------------------------------------------------------------------------------*/


void uart_rx_callback(circ_buffer_t *buffer_rx){

    /* Copiem el contingut del buffer a partir de la posici� i = 1, ja que la posici� buffer[0] cont� el delimitador de
     * final de seq��ncia (al ser un buffer circular, aquest cont� delimitador '\0' al principi i al final del vector). Les
     * dades les desarem a la variable global "uart-string". No cal protegir la escriptura sobre la variable global
     * amb cap m�tex perqu� nom�s es modifica des d'aquesta ISR, que s'executa amb prioritat elevada (kernel) i a m�s
     * no hi haur� cap altra tasca que pugui alterar la secci� cr�tica (variable global) mentre aquesta es modifica per la ISR. */

    uint8_t i;
    for (i=1;i<buffer_rx->end;i++){
        uart_command[i-1] = (char)buffer_rx->buffer[i];
    }

    /* Delimitem uart_string amb el car�cter final de cadena */
    uart_command[i-1] = '\0';

    /* Reinicialitzem el buffer_rx per tal de poder capturar la seg�ent comanda */
    buffer_rx->end = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Entrega el sem�for para desbloquejar la tasca prvUARTReadTask */
    xSemaphoreGiveFromISR(xRunUARTReadingTaskSemaphore, &xHigherPriorityTaskWoken);

    /* Pasa xHigherPriorityTaskWoken a portYIELD_FROM_ISR() */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void get_adc_results (adc_result results_buffer ){
    accel_values.x = results_buffer[0];
    accel_values.y = results_buffer[1];
    accel_values.z = results_buffer[2];
    xSemaphoreGiveFromISR( xADCReadingAvailableSemaphore, NULL );
}


/*------------------------------------------------------------------------------

                                FUNCIONS AUXILIARS

------------------------------------------------------------------------------*/


/* Funci� que utilitzem per a convertir al tipus demanat (command_t) els car�cters rebuts per UART a la variable global
 * de tipus string (uart_string). S'implementa control d'errors b�sic (format incorrecte de comanda i magnitud dels
 * valors introdu�ts */

command_t uart_to_command(void)
{
    uint8_t n = 0;
    char tmp_str[10];
    tmp_str[0] = '\0';
    command_t command;

    while (uart_command[n] != ',' & uart_command[n] != '\0')
    {
        strncat(tmp_str, &uart_command[n], 1);
        n++;
    }

    /* Control d'errors (format): Si no s'ha trobat una coma o un signe negatiu, aleshores la comanda
     * est� mal formatada. En aquest cas retornem l'enter "99" (COMMAND_ERROR), xifra que utilitzarem com a codi d'error */
    if (uart_command[n] != ',' && uart_command[n] != '-')
    {
        command.x = COMMAND_ERROR;
        command.y = COMMAND_ERROR;
    }
    else
    {
        command.x = atoi(tmp_str);

        tmp_str[0] = '\0';
        n++;
        while (uart_command[n] != '\0')
        {
            strncat(tmp_str, &uart_command[n], 1);
            n++;
        }
        command.y = atoi(tmp_str);
    }

    /* Control d'errors (magnitud): Si les coordenades son inferiors o superiors al rang demanat, retornem codi d'error */
    if (command.x < -45 || command.x > 45 || command.y < -45 || command.y > 45)
    {
        command.x = COMMAND_ERROR;
        command.y = COMMAND_ERROR;
    }

    return command;
}
