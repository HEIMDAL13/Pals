/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "driverlib.h"

#include "grlib.h"
#include "button.h"
#include "imageButton.h"
#include "radioButton.h"
#include "checkbox.h"
#include "LcdDriver/kitronix320x240x16_ssd2119_spi.h"
#include "images/images.h"
#include "touch_P401R.h"

//Pin 3.2 is RXD
//Pin 3.3 is TXD

//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};
//![Simple UART Config]

#define MSG_LENGTH 100
#define TRUE 1
#define FALSE 0

#define ACK_CHAR 'K'
#define NACK_CHAR 'N'
#define DEL_MSG '\n'

#define TEXT_MIDDLE 160

#define TEXT_CENTERED 0
#define TEXT_LEFT 1

#define NUM_PARAMS 5

#define FONT_STYLE_NORMAL 0
#define FONT_STYLE_BOLD 1
#define FONT_STYLE_ITALIC 2

#define FONT_TYPE_S 0
#define FONT_TYPE_SS 1
#define FONT_TYPE_SC 2
#define FONT_TYPE_TT 3

#define BTN_COLOR_DAY GRAPHICS_COLOR_RED
#define BTN_SELECTCOLOR_DAY GRAPHICS_COLOR_ORANGE
#define BTN_BORDERCOLOR_DAY GRAPHICS_COLOR_RED
#define BTN_TEXTCOLOR_DAY GRAPHICS_COLOR_BLACK
#define BTN_SELTEXTCOLOR_DAY GRAPHICS_COLOR_BLACK

#define BTN_COLOR_NIGHT GRAPHICS_COLOR_RED
#define BTN_SELECTCOLOR_NIGHT GRAPHICS_COLOR_ORANGE
#define BTN_BORDERCOLOR_NIGHT GRAPHICS_COLOR_RED
#define BTN_TEXTCOLOR_NIGHT GRAPHICS_COLOR_RED
#define BTN_SELTEXTCOLOR_NIGHT GRAPHICS_COLOR_BLACK

#define CKBX_COLOR_DAY GRAPHICS_COLOR_RED
#define CKBX_SELECTCOLOR_DAY GRAPHICS_COLOR_BLACK
#define CKBX_TEXTCOLOR_DAY GRAPHICS_COLOR_BLACK

#define CKBX_COLOR_NIGHT GRAPHICS_COLOR_RED
#define CKBX_SELECTCOLOR_NIGHT GRAPHICS_COLOR_BLACK
#define CKBX_TEXTCOLOR_NIGHT GRAPHICS_COLOR_RED

#define BACKGROUND_COLOR_DAY GRAPHICS_COLOR_WHITE
#define FOREGROUND_COLOR_DAY GRAPHICS_COLOR_BLACK

#define BACKGROUND_COLOR_NIGHT GRAPHICS_COLOR_BLACK
#define FOREGROUND_COLOR_NIGHT GRAPHICS_COLOR_RED

#define WINDOW_MAIN 0
#define WINDOW_TEST 1
#define WINDOW_OPTIONS 2

#define TEXT_SIZE_NORMAL 20
#define TEXT_SIZE_BIG 32

#define DEBUG_DEFAULT TRUE

#define WARNING_OK 0
#define WARNING_1 1
#define WARNING_2 2
#define WARNING_3 3

#define COMMAND_1 0
#define COMMAND_2 1

int warningFlag;

void Delay(uint16_t msec);
void boardInit(void);
void clockInit(void);
void initializeObjects(int isDayMode);
void drawWindow(int id_window);
void drawText(int8_t *message, int posX, int posY, int textSize, int text_justification);
void resetDisplay(int isDayMode);


void manageDisplay(int windowIdentity);
void append_char(char* s, char c);
void splitCmd(char* s, float params []);
Graphics_Font getFont(int font_type, int font_size, int font_style);
Graphics_Button initializeBut(int8_t *message, int xmin, int xmax, int ymin, int ymax, int text_x, int text_y, int dayMode);
Graphics_CheckBox initializeCkbx(int8_t *message, int xPos, int yPos, int dayMode);
void updateFont(int font_type, int text_size, int font_style);
char * constructText(char * msg, int number);

const char CMD_1 [] = {"CMD_1"};
const char CMD_2 [] = {"CMD_2"};

char message [MSG_LENGTH];
int cmd_receive = FALSE;

touch_context g_sTouchContext;

Graphics_Button btnTest;
Graphics_Button btnOptions;
Graphics_Button btnOk;
Graphics_CheckBox ckbxDayOrNight;
Graphics_CheckBox ckbxDebug;

// Graphic library context
Graphics_Context g_sContext;

uint8_t font_type;
uint8_t font_style;
Graphics_Font font_glob;

int isDayMode;
int windowIdentity;
int debug;

int processing;
int command = -1;

float params [NUM_PARAMS];
int temp_params [NUM_PARAMS] = {-1, -1, -1, -1, -1};


int main(void){
    /* Initialize the demo. */
    boardInit();
    clockInit();

    /* Globally enable interrupts. */
    __enable_interrupt();

    // LCD setup using Graphics Library API calls
    Kitronix320x240x16_SSD2119Init();
    Graphics_initContext(&g_sContext, &g_sKitronix320x240x16_SSD2119);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    warningFlag = WARNING_OK;
    processing = FALSE;

    debug = DEBUG_DEFAULT;
    windowIdentity = WINDOW_MAIN;
    isDayMode = TRUE;
    initializeObjects(isDayMode);

    font_type = FONT_TYPE_S;
    font_style = FONT_STYLE_ITALIC;
    updateFont(font_type, TEXT_SIZE_NORMAL, font_style);

    Graphics_clearDisplay(&g_sContext);
    touch_initInterface();
    drawWindow(windowIdentity);

    /* Halting WDT  */
    WDT_A_hold(WDT_A_BASE);

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    //![Simple UART Example]
    /* Configuring UART Module */
    UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    MAP_Interrupt_enableMaster();

    while(1){
        if(processing == TRUE && windowIdentity == WINDOW_TEST){
            int i;
            for(i=0; i<NUM_PARAMS; ++i){
                temp_params[i] = params[i];
            }

            manageDisplay(WINDOW_TEST);
        }

        touch_updateCurrentTouch(&g_sTouchContext);
        if(g_sTouchContext.touch){
            manageDisplay(windowIdentity);
        }
    }
}

void manageDisplay(int windowIdentity){
    if(windowIdentity == WINDOW_MAIN){
        if(Graphics_isButtonSelected(&btnTest, g_sTouchContext.x, g_sTouchContext.y)){
            Graphics_drawSelectedButton(&g_sContext, &btnTest);
            drawWindow(WINDOW_TEST);
        }
        else if(Graphics_isButtonSelected(&btnOptions, g_sTouchContext.x, g_sTouchContext.y)){
            Graphics_drawSelectedButton(&g_sContext, &btnOptions);
            drawWindow(WINDOW_OPTIONS);
        }
    }
    else if(windowIdentity == WINDOW_OPTIONS){
        if (Graphics_isCheckBoxSelected(&ckbxDayOrNight, g_sTouchContext.x, g_sTouchContext.y)){
            if(ckbxDayOrNight.selected == FALSE){
                ckbxDayOrNight.selected = TRUE;
                isDayMode = FALSE;

                ckbxDayOrNight.textColor = CKBX_TEXTCOLOR_NIGHT;
                ckbxDebug.textColor = CKBX_TEXTCOLOR_NIGHT;
            }
            else if(ckbxDayOrNight.selected == TRUE){
                ckbxDayOrNight.selected = FALSE;
                isDayMode = TRUE;

                ckbxDayOrNight.textColor = CKBX_TEXTCOLOR_DAY;
                ckbxDebug.textColor = CKBX_TEXTCOLOR_DAY;
            }

            drawWindow(WINDOW_OPTIONS);
        }
        else if(Graphics_isCheckBoxSelected(&ckbxDebug, g_sTouchContext.x, g_sTouchContext.y)){
            if (ckbxDebug.selected == FALSE){
                ckbxDebug.selected = TRUE;
                debug = TRUE;

                Graphics_drawSelectedCheckBox(&g_sContext, &ckbxDebug);
            }
            else if (ckbxDebug.selected == TRUE){
                ckbxDebug.selected = FALSE;
                debug = FALSE;

                Graphics_drawCheckBox(&g_sContext, &ckbxDebug);
            }

        }
        else if(Graphics_isButtonSelected(&btnOk, g_sTouchContext.x, g_sTouchContext.y)){
            Graphics_drawSelectedButton(&g_sContext, &btnOk);
            drawWindow(WINDOW_MAIN);
        }
    }
    else if(windowIdentity == WINDOW_TEST){
        drawWindow(WINDOW_TEST);
    }
}

void initializeObjects(int isDayMode){

    btnTest = initializeBut("Test", 48, 138, 160, 200, 75, 170, isDayMode);
    btnOptions = initializeBut("Options", 178, 268, 160, 200, 190, 170, isDayMode);
    btnOk = initializeBut("OK", 115, 205, 180, 220, 145, 190, isDayMode);

    ckbxDayOrNight = initializeCkbx("Nightmode", 30, 80, isDayMode);
    ckbxDebug = initializeCkbx("Debug", 30, 130, isDayMode);
}

void resetDisplay(int isDayMode){
    if(isDayMode){
        Graphics_setForegroundColor(&g_sContext, FOREGROUND_COLOR_DAY);
        Graphics_setBackgroundColor(&g_sContext, BACKGROUND_COLOR_DAY);
    }
    else{
        Graphics_setForegroundColor(&g_sContext, FOREGROUND_COLOR_NIGHT);
        Graphics_setBackgroundColor(&g_sContext, BACKGROUND_COLOR_NIGHT);
    }

    Graphics_clearDisplay(&g_sContext);
}

void drawWindow(int id_window){
    if(id_window == WINDOW_MAIN){
        windowIdentity = WINDOW_MAIN;
        resetDisplay(isDayMode);

        drawText("Welcome to the CarPal Demo!", TEXT_MIDDLE, 30, TEXT_SIZE_NORMAL, TEXT_CENTERED);
        drawText("What do you want to do?", TEXT_MIDDLE, 110, TEXT_SIZE_NORMAL, TEXT_CENTERED);

        Graphics_drawButton(&g_sContext, &btnTest);
        Graphics_drawButton(&g_sContext, &btnOptions);
    }
    else if (id_window == WINDOW_OPTIONS){
        windowIdentity = WINDOW_OPTIONS;
        resetDisplay(isDayMode);

        drawText("Options", TEXT_MIDDLE, 30, TEXT_SIZE_BIG, TEXT_CENTERED);

        updateFont(font_type, TEXT_SIZE_NORMAL, font_style);
        Graphics_drawCheckBox(&g_sContext, &ckbxDayOrNight);
        Graphics_drawCheckBox(&g_sContext, &ckbxDebug);
        Graphics_drawButton(&g_sContext, &btnOk);
    }
    else if (id_window == WINDOW_TEST){
        if(windowIdentity == WINDOW_MAIN){
            resetDisplay(isDayMode);
            windowIdentity = WINDOW_TEST;
        }

        if(warningFlag == WARNING_OK){
            drawText("   Everything OK   ", TEXT_MIDDLE, 70, TEXT_SIZE_BIG, TEXT_CENTERED);
        }
        else if(warningFlag == WARNING_1){
            drawText("       Head heavily tilted!       ", TEXT_MIDDLE, 70, TEXT_SIZE_BIG, TEXT_CENTERED);
        }
        else if(warningFlag == WARNING_2){
            drawText("       Blinking duration!       ", TEXT_MIDDLE, 70, TEXT_SIZE_BIG, TEXT_CENTERED);
        }
        else if(warningFlag == WARNING_3){
            drawText("       Blinking rate!       ", TEXT_MIDDLE, 70, TEXT_SIZE_BIG, TEXT_CENTERED);
        }

        if(debug == TRUE){
            char msg[MSG_LENGTH];
            sprintf(msg, "Blinks: %.2f", params[0]);
            drawText(msg, 10, 160, TEXT_SIZE_NORMAL, TEXT_LEFT);
            sprintf(msg, "Avg. blink rates: %.2f", params[2]);
            drawText(msg, 10, 180, TEXT_SIZE_NORMAL, TEXT_LEFT);
            sprintf(msg, "Avg. blink duration: %.2f", params[4]);
            drawText(msg, 10, 200, TEXT_SIZE_NORMAL, TEXT_LEFT);

        }

        processing = FALSE;
    }
}

void drawText(int8_t *message, int posX, int posY, int textSize, int text_justification){

    updateFont(font_type, textSize, font_style);

    if(text_justification == TEXT_CENTERED){
        Graphics_drawStringCentered(&g_sContext, message, AUTO_STRING_LENGTH, TEXT_MIDDLE, posY, 1);
    }
    else if(text_justification == TEXT_LEFT){
        Graphics_drawString(&g_sContext, message, AUTO_STRING_LENGTH, posX, posY, 1);
    }

}

void updateFont(int font_type, int text_size, int font_style){
    font_glob = getFont(font_type, text_size, font_style);
    Graphics_setFont(&g_sContext, &font_glob);
}

Graphics_Font getFont(int font_type, int font_size, int font_style){
    if(font_type < 0 || font_type > 3 || font_style < 0 || font_style > 2){
        return g_sFontCm20;
    }
    else
    {
        if (font_type == FONT_TYPE_SC){
            if(font_size == TEXT_SIZE_NORMAL){
                return g_sFontCmsc20;
            }
            else if(font_size == TEXT_SIZE_BIG){
                return g_sFontCmsc32;
            }

            return g_sFontCmsc20;
        }
        else if (font_type == FONT_TYPE_TT){
            if (font_size == TEXT_SIZE_NORMAL){
                return g_sFontCmtt20;
            }
            else if (font_size == TEXT_SIZE_BIG){
                return g_sFontCmtt32;
            }

            return g_sFontCmtt20;
        }
        else if (font_type == FONT_TYPE_S){
            if (font_size == TEXT_SIZE_NORMAL){
                switch (font_style){
                case (FONT_STYLE_NORMAL):
                    return g_sFontCm20;
                case (FONT_STYLE_BOLD):
                    return g_sFontCm20b;
                case (FONT_STYLE_ITALIC):
                    return g_sFontCm20i;
                }
            }

            else if (font_size == TEXT_SIZE_BIG){
                switch (font_style){
                case (FONT_STYLE_NORMAL):
                    return g_sFontCm32;
                case (FONT_STYLE_BOLD):
                    return g_sFontCm32b;
                case (FONT_STYLE_ITALIC):
                    return g_sFontCm32i;
                }
            }

        }
        else if (font_type == FONT_TYPE_SS){
            if (font_size == TEXT_SIZE_NORMAL){
                switch (font_style){
                case (FONT_STYLE_NORMAL):
                    return g_sFontCmss20;
                case (FONT_STYLE_BOLD):
                    return g_sFontCmss20b;
                case (FONT_STYLE_ITALIC):
                    return g_sFontCmss20i;
                }
            }
            else if (font_size == TEXT_SIZE_BIG){
                switch (font_style){
                case (FONT_STYLE_NORMAL):
                    return g_sFontCmss32;
                case (FONT_STYLE_BOLD):
                    return g_sFontCmss32b;
                case (FONT_STYLE_ITALIC):
                    return g_sFontCmss32i;
                }
            }

            return g_sFontCmss20;
        }
    }

    return g_sFontCm20;
}



Graphics_Button initializeBut(int8_t *message, int xmin, int xmax, int ymin, int ymax, int text_x, int text_y, int dayMode){
    Graphics_Button button;

    button.xMin = xmin;
    button.xMax = xmax;
    button.yMin = ymin;
    button.yMax = ymax;
    button.borderWidth = 1;
    button.selected = FALSE;
    button.textXPos = text_x;
    button.textYPos = text_y;
    button.font = &font_glob;
    button.text = message;

    if(dayMode == TRUE){
        button.fillColor = BTN_COLOR_DAY;
        button.borderColor = BTN_BORDERCOLOR_DAY;
        button.selectedColor = BTN_SELECTCOLOR_DAY;
        button.textColor = BTN_TEXTCOLOR_DAY;
        button.selectedTextColor = BTN_SELTEXTCOLOR_DAY;
    }
    else{
        button.fillColor = BTN_COLOR_NIGHT;
        button.borderColor = BTN_BORDERCOLOR_NIGHT;
        button.selectedColor = BTN_SELECTCOLOR_NIGHT;
        button.textColor = BTN_TEXTCOLOR_NIGHT;
        button.selectedTextColor = BTN_SELTEXTCOLOR_NIGHT;
    }

    return button;
}

Graphics_CheckBox initializeCkbx(int8_t *message, int xPos, int yPos, int dayMode){
    Graphics_CheckBox box;

    box.xPosition = xPos;
    box.yPosition = yPos;
    box.gap = 15;
    box.selected = FALSE;
    box.text = message;
    box.font = &font_glob;

    if (dayMode == TRUE){
        box.backgroundColor = CKBX_COLOR_DAY;
        box.selectedColor = CKBX_SELECTCOLOR_DAY;
        box.textColor = CKBX_TEXTCOLOR_DAY;
    }
    else{
        box.backgroundColor = CKBX_COLOR_NIGHT;
        box.selectedColor = CKBX_SELECTCOLOR_NIGHT;
        box.textColor = CKBX_TEXTCOLOR_NIGHT;
    }

    return box;
}

void boardInit()
{
    FPU_enableModule();
}

void clockInit(void)
{
    /* 2 flash wait states, VCORE = 1, running off DC-DC, 48 MHz */
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    PCM_setPowerState(PCM_AM_DCDC_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_setDCOFrequency(48000000);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, 1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, 1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, 1);

    return;
}

void Delay(uint16_t msec){
    uint32_t i = 0;
    uint32_t time = (msec / 1000) * (SYSTEM_CLOCK_SPEED / 15);

    for(i = 0; i < time; i++)
    {
        ;
    }
}

void EUSCIA2_IRQHandler(void){
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG){
        if (processing){
            char b = MAP_UART_receiveData(EUSCI_A2_BASE);
            message[0] = '\0';
            MAP_UART_transmitData(EUSCI_A2_BASE, NACK_CHAR);
        }
        else{
            char c = MAP_UART_receiveData(EUSCI_A2_BASE);
            if(c == DEL_MSG){

                //is it the command line?
                if(strcmp(message, CMD_1) == 0){
                    command = COMMAND_1;
                }
                else if(strcmp(message, CMD_2) == 0){
                    command = COMMAND_2;
                }
                else if(command == COMMAND_1){
                    warningFlag = atoi(message);
                    processing = TRUE;
                }
                else if(command == COMMAND_2){
                    splitCmd(message, params);
                    processing = TRUE;
                }

                message[0] = '\0';
                MAP_UART_transmitData(EUSCI_A2_BASE, ACK_CHAR);
            }
            else{
                append_char(message, c);
                MAP_UART_transmitData(EUSCI_A2_BASE, ACK_CHAR);
            }
        }
    }
}

void append_char(char* s, char c) {
    int len = strlen(s);
    s[len] = c;
    s[len+1] = '\0';
}

void splitCmd(char* s, float params []){
    int param_index = 0;

    char current_param [10];
    current_param[0] = '\0';
    int i = 0;
    for(i=0; i<strlen(s); ++i){
        if(s[i] == '&'){
            params[param_index] = atof(current_param);
            ++param_index;
            current_param[0] = '\0';
        }
        else if(i == strlen(s)-1){
            append_char(current_param, s[i]);
            params[param_index] = atof(current_param);
        }
        else{
            append_char(current_param, s[i]);
        }
    }
}
