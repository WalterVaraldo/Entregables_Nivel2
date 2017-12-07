/* Copyright 2014, ChaN
 * Copyright 2016, Matias Marando
 * Copyright 2016, Eric Pernia
 * Copyright 2017, Agustin Bassi
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inlcusiones]============================================*/

#include "sapi.h"     // <= Biblioteca sAPI
//#include "ff.h"       // <= Biblioteca FAT FS

/*==================[definiciones y macros]==================================*/

#define ADDRESS_MAG  0x0D  //direccion magnetometro
#define FILENAME                "Magnetómetro_datalogger.txt"
#define MAX_BYTES_TO_SAVE       100
#define BYTES_OF_INFORMATION    35

/*==================[definiciones de datos internos]=========================*/

CONSOLE_PRINT_ENABLE

// <-- FatFs work area needed for each volume
static FATFS    fs;     

// <-- File object needed for each open file

static FIL      fp;  
// Arreglo donde se almacena la hora y fecha

static char     infoToSave [BYTES_OF_INFORMATION];

// Variables del magnetometro (hmc5883l)

static int16_t  hmc5883l_x_raw = 0;
static int16_t  hmc5883l_y_raw = 0;
static int16_t  hmc5883l_z_raw = 0;

/*==================[definiciones de datos externos]=========================*/


/*==================[declaraciones de funciones internas]====================*/

// Puntero, cada vez que ocurre un diskTickHook
bool_t      diskTickHook    ( void *ptr );

// comandos para enviar por UART 
uint32_t    f_mount_uart    ( FATFS *fs, char * alternativeText, uint32_t mountParameters);   
uint32_t    f_open_uart     ( FATFS *fs, char * fileName, uint32_t openParameters);
void        f_write_uart    ( FATFS *fs, char * messageToWrite, uint32_t bytesToWrite, uint32_t * bytesWritten); 
void        f_close_uart    ( FATFS *fs);

// Formato de fecha, hora y el valor de los ejes del magnetometro x, y, z.
void formatInfoToSave( rtc_t * rtc );

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){
// Definicion de variables locales usadas solo en el main.
uint32_t            nbytes;
rtc_t               rtc;
delay_t             delay1s;
HMC5883L_config_t   hmc5883L_configValue;
bool_t              flagLogActive = FALSE;
    
    // Configura el core y los perifericos que se van a utilizar.
    boardConfig ();    
    spiConfig   ( SPI0 );   
    uartConfig  ( UART_USB, 115200);    
    tickConfig  ( 10, diskTickHook );  
    delayConfig ( &delay1s, 1000 );
    i2cConfig(I2C0, 100000);
    
    // Configuraciones especificas para el modulo HMC5883
    hmc5883lPrepareDefaultConfig( &hmc5883L_configValue );
    hmc5883L_configValue.mode    = HMC5883L_continuous_measurement;
    hmc5883L_configValue.samples = HMC5883L_8_sample;
    hmc5883lConfig( hmc5883L_configValue );
    bufferTransmision[0] = 0x0B;  //registro a escribir 0x0B
    bufferTransmision[1] = 1;  //valor a escribir en registro (set/reset period)
    i2cWrite( I2C0, ADDRESS_MAG, bufferTransmision, 2, TRUE );  //escribe en registro

    // Setea hora y fecha e inicia el rtc con rtcConfig(&rtc);
    rtc.year  = 2017;
    rtc.month = 7;
    rtc.mday  = 3;
    rtc.wday  = 1;
    rtc.hour  = 13;
    rtc.min   = 17;
    rtc.sec   = 0;     
    rtcConfig( &rtc );
   
    // Configura la memoria SD. 
    // Esto es para mandar los datos por la UART usando las funciones de FAT.
    // El motivo es que muchos modulos SD no funcionan correctamente.
    
    if( f_mount( &fs, "", 0 ) != FR_OK ){
    }

    if( f_open( &fp, FILENAME, FA_WRITE | FA_OPEN_APPEND ) == FR_OK ){
    
        f_write( &fp, "Log HMC5883 con timestamp en memoria SD.\n\r", MAX_BYTES_TO_SAVE, &nbytes );
       
        // Si se pudo abrir el archivo enciende el led VERDE.
        gpioWrite( LEDG, ON );    
    } else{
        // Si se pudo abrir el archivo enciende el led ROJO.
        gpioWrite( LEDR, ON );
    }
    
    //REPETIR POR SIEMPRE.
    while( 1 ){
        // Si paso un segundo entra en el if.
        if( delayRead( &delay1s ) ){
            
            // Si esta activado el log (se setea con TEC1).
            if (flagLogActive){
                // Actualiza en las variables hmc5883l_x,y,z_raw el valor del magnetometro en XYZ.
                hmc5883lRead( &hmc5883l_x_raw, &hmc5883l_y_raw, &hmc5883l_z_raw );
                
                // Actualiza el clock a la nueva hora.
                rtcRead( &rtc );
                
                // Formatea la salida y la guarda en el arreglo global informationToSave.
                // Formato: DD/MM/YYYY_HH:MM:SS_XXX;YYY;ZZZ;
                formatInfoToSave( &rtc );
                
                // Escribe en la memoria SD el array formateado con el timestamp y los ejes X; Y; Z.
                f_write( &fp, infoToSave, MAX_BYTES_TO_SAVE, &nbytes );
                
                // Una vez que escribio cierra el archivo.
                f_close(&fp);    
            }
        }
        
        // Con TEC1 se activa/desactiva el log.
        if (!gpioRead(TEC1)){
            flagLogActive = !flagLogActive;
            delay (1000);
        }
    } 
    
    
    return 0;
}
/*==================[definiciones de funciones internas]=====================*/

// Formatea el arreglo a guardar con DD/MM/YYYY_HH:MM:SS_XXX;YYY;ZZZ;
void        formatInfoToSave ( rtc_t * rtc ){
    infoToSave[0]  = (rtc->mday/10) + '0';
    infoToSave[1]  = (rtc->mday%10) + '0';
    infoToSave[2]  = '/';
    infoToSave[3]  = (rtc->month/10) + '0';
    infoToSave[4]  = (rtc->month%10) + '0';
    infoToSave[5]  = '/';
    infoToSave[6]  = (rtc->year/1000) + '0';
    infoToSave[7]  = ((rtc->year%1000)/100) + '0';
    infoToSave[8]  = ((rtc->year%100)/10) + '0';
    infoToSave[9]  = (rtc->year%10) + '0';
    infoToSave[10] = '_';
    infoToSave[11] = (rtc->hour/10) + '0';
    infoToSave[12] = (rtc->hour%10) + '0';
    infoToSave[13] = ':';
    infoToSave[14] = (rtc->min/10) + '0';
    infoToSave[15] = (rtc->min%10) + '0';
    infoToSave[16] = ':';
    infoToSave[17] = (rtc->sec/10) + '0';
    infoToSave[18] = (rtc->sec%10) + '0';
    infoToSave[19] = '_';
    infoToSave[20] = (uint8_t)(hmc5883l_x_raw/100) + '0';
    infoToSave[21] = (uint8_t)(hmc5883l_x_raw%100)/10 + '0';
    infoToSave[22] = (uint8_t)(hmc5883l_x_raw%10) + '0';
    infoToSave[23] = ';';
    infoToSave[24] = (uint8_t)(hmc5883l_y_raw/100) + '0';
    infoToSave[25] = (uint8_t)(hmc5883l_y_raw%100)/10 + '0';
    infoToSave[26] = (uint8_t)(hmc5883l_y_raw%10) + '0';
    infoToSave[27] = ';';
    infoToSave[28] = (uint8_t)(hmc5883l_z_raw/100) + '0';
    infoToSave[29] = (uint8_t)(hmc5883l_z_raw%100)/10 + '0';
    infoToSave[30] = (uint8_t)(hmc5883l_z_raw%10) + '0';
    infoToSave[31] = ';';
    infoToSave[32] = '\n';
    infoToSave[33] = '\r';
    infoToSave[34] = '\0';
}

// Función con UART para ver lo que se está registrando

uint32_t    f_mount_uart    ( FATFS *fs, char * alternativeText, uint32_t mountParameters){
    uartWriteString(UART_USB, "\n\rMontando el file system.\n\r");
    return FR_OK;
}

uint32_t    f_open_uart     ( FATFS *fs, char * fileName, uint32_t openParameters){
uint8_t counter;
    
    uartWriteString(UART_USB, "Abriendo archivo: ");
    uartWriteString(UART_USB, FILENAME);
    uartWriteString(UART_USB, "\n\r");
    
//---------------[Elige los parametros con los que se abriria el archivo]---------------//
    
    if (openParameters == (FA_WRITE | FA_CREATE_ALWAYS)){
        for (counter = 0; counter < 10; counter++){
            uartWriteString(UART_USB, "\n\r");   
        }
    } else if (openParameters == (FA_WRITE | FA_OPEN_APPEND)){
        uartWriteString(UART_USB, "\n\r");    
    }
    return FR_OK;
}
void        f_write_uart    ( FATFS *fs, char * messageToWrite, uint32_t bytesToWrite, uint32_t * bytesWritten){
uint8_t counter;
char * messagePointer = messageToWrite;    

    *bytesWritten = bytesToWrite;
    for (counter = 0; counter < bytesToWrite && *messagePointer != '\0'; counter++, messagePointer++){
        uartWriteByte(UART_USB, *messagePointer);
    }
}

void        f_close_uart    ( FATFS *fs){
    uartWriteString(UART_USB, "Cerrando archivo: ");
    uartWriteString(UART_USB, FILENAME);
    uartWriteString(UART_USB, "\n\r");
}

/*==================[definiciones de funciones externas]=====================*/

bool_t diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
   return TRUE;
}


/*==================[fin del archivo]========================================*/
