/*
 * @brief This file contains USB HID Keyboard example using USB ROM Drivers.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "sapi.h"
#include <stdint.h>
#include <string.h>
#include "usbd_rom_api.h"
#include "hid_keyboard.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/**
 * @brief Structure to hold Keyboard data
 */
typedef struct {
	USBD_HANDLE_T hUsb;	/*!< Handle to USB stack. */
	uint8_t report[KEYBOARD_REPORT_SIZE];	/*!< Last report data  */
	uint8_t tx_busy;	/*!< Flag indicating whether a report is pending in endpoint queue. */
} Keyboard_Ctrl_T;

/** Singleton instance of Keyboard control */
static Keyboard_Ctrl_T g_keyBoard;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

extern const uint8_t Keyboard_ReportDescriptor[];
extern const uint16_t Keyboard_ReportDescSize;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
uint16_t key = 0;

// Pines conectados a las Filas --> Salidas (MODO = OUTPUT)
uint8_t keypadRowPins[4] = {
   RS232_TXD, // Row 0
   CAN_RD,    // R1
   CAN_TD,    // R2
   T_COL1     // R3
};

// Pines conectados a las Columnas --> Entradas con pull-up (MODO = INPUT_PULLUP)
uint8_t keypadColPins[4] = {
   T_FIL0,    // Column 0
   T_FIL3,    // C1
   T_FIL2,    // C2
   T_COL0     // C3
};


void configurarTecladoMatricial(void);
bool_t leerTecladoMatricial( void );



/* Routine to update keyboard state */
static void Keyboard_UpdateReport(void){
    
    configurarTecladoMatricial();


    
    HID_KEYBOARD_CLEAR_REPORT(&g_keyBoard.report[0]);
    
if( leerTecladoMatricial() ){
    
    switch(key){
        
        case 0:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x1E); //1
        break;
        
        case 1:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x1F); //2
        break;
        
        case 2:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x20); //3
        break;
        
        case 3:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x04); //A
        break;
        
        case 4:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x21); //4
        break;
        
        case 5:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x22); //5
        break;
        
        case 6:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x23); //6
        break;
        
        case 7:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x05); //B
        break;
        
        case 8:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x24); //7
        break;
        
        case 9:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x25); //8
        break;
        
        case 10:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x26); //9
        break;
        
        case 11:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x06); //C
        break;
        
        case 12:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x39); //* = Caps Lock
        break;
        
        case 13:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x27); //0
        break;
        
        case 14:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x28); //# = Enter
        break;
        
        case 15:
            HID_KEYBOARD_REPORT_SET_KEY_PRESS(g_keyBoard.report, 0x07); //D
        break;
    
   
   }

}

}


void configurarTecladoMatricial( void ){
   
   uint8_t i = 0;
   
   // Configure Rows as Outputs
   for( i=0; i<4; i++ ){
      gpioConfig( keypadRowPins[i], GPIO_OUTPUT );
   }

   // Configure Columns as Inputs with pull-up resistors enable
   for( i=0; i<4; i++ ){
      gpioConfig( keypadColPins[i], GPIO_INPUT_PULLUP );
   }
}



bool_t leerTecladoMatricial( void ){
    


   bool_t retVal = FALSE;

   uint16_t r = 0; // Rows
   uint16_t c = 0; // Columns

   // Poner todas las filas en estado BAJO
   for( r=0; r<4; r++ ){
	  gpioWrite( keypadRowPins[r], LOW );
   }

   // Chequear todas las columnas buscando si hay alguna tecla presionada
   for( c=0; c<4; c++ ){

      // Si leo un estado BAJO en una columna entonces puede haber una tecla presionada
      if( !gpioRead( keypadColPins[c] ) ){

         delay( 50 ); // Anti-rebotes de 50 ms

         // Poner todas las filas en estado ALTO excepto la primera
         for( r=1; r<4; r++ ){
            gpioWrite( keypadRowPins[r], HIGH );
         }

         // Buscar que tecla esta presionada
         for( r=0; r<4; r++ ){

            // Poner la Fila[r-1] en estado ALTO y la Fila[r] en estado BAJO
            if( r>0 ){ // Exceptua el indice negativo en el array
               gpioWrite( keypadRowPins[r-1], HIGH );
            }
            gpioWrite( keypadRowPins[r], LOW );

            // Chequear la Columna[c] en Fila[r] para buscar si la tecla esta presionada
            // Si dicha tecla esta oresionada (en estado BAJO) entonces retorna
            // graba la tecla en key y retorna TRUE
            if( !gpioRead( keypadColPins[c] ) ){
               retVal = TRUE;
               key = r * 4 + c;
               /*
                  Formula de las teclas de Teclado Matricial (Keypad)
                  de 4 filas (rows) * 5 columnas (columns)

                     c0 c1 c2 c3 c4
                  r0  0  1  2  3  4
                  r1  5  6  7  8  9   Si se presiona la tecla r[i] c[j]:
                  r2 10 11 12 13 14   valor = (i) * cantidadDeColumnas + (j)
                  r3 15 16 17 18 19
               */
               return retVal;
            }
         }

      }
   }
   return retVal;
}



/* HID Get Report Request Callback. Called automatically on HID Get Report Request */
static ErrorCode_t Keyboard_GetReport( USBD_HANDLE_T hHid,
                                       USB_SETUP_PACKET *pSetup,
                                       uint8_t * *pBuffer,
                                       uint16_t *plength ){

	/* ReportID = SetupPacket.wValue.WB.L; */
	switch (pSetup->wValue.WB.H) {

      case HID_REPORT_INPUT:
         Keyboard_UpdateReport();
         memcpy(*pBuffer, &g_keyBoard.report[0], KEYBOARD_REPORT_SIZE);
         *plength = KEYBOARD_REPORT_SIZE;
      break;

      case HID_REPORT_OUTPUT:				/* Not Supported */
      case HID_REPORT_FEATURE:			/* Not Supported */

         return ERR_USBD_STALL;
	}

	return LPC_OK;
}

/* HID Set Report Request Callback. Called automatically on HID Set Report Request */
static ErrorCode_t Keyboard_SetReport( USBD_HANDLE_T hHid,
                                       USB_SETUP_PACKET *pSetup,
                                       uint8_t * *pBuffer,
                                       uint16_t length){

   /* we will reuse standard EP0Buf */
   if (length == 0) {
      return LPC_OK;
   }

   /* ReportID = SetupPacket.wValue.WB.L; */
   switch (pSetup->wValue.WB.H){

      case HID_REPORT_OUTPUT:
         /*  If the USB host tells us to turn on the NUM LOCK LED,
          *  then turn on LED#2.
          */
         if (**pBuffer & 0x01) {
            gpioWrite( LEDB, ON );
         }
         else {
            gpioWrite( LEDB, OFF);
         }
      break;

      case HID_REPORT_INPUT:				/* Not Supported */
      case HID_REPORT_FEATURE:			/* Not Supported */

         return ERR_USBD_STALL;
   }

   return LPC_OK;
}

/* HID interrupt IN endpoint handler */
static ErrorCode_t Keyboard_EpIN_Hdlr( USBD_HANDLE_T hUsb,
                                       void *data,
                                       uint32_t event ){
   switch (event) {
      case USB_EVT_IN:
         g_keyBoard.tx_busy = 0;
         break;
   }
   return LPC_OK;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* HID keyboard init routine */
ErrorCode_t Keyboard_init(USBD_HANDLE_T hUsb,
						  USB_INTERFACE_DESCRIPTOR *pIntfDesc,
						  uint32_t *mem_base,
						  uint32_t *mem_size){

   USBD_HID_INIT_PARAM_T hid_param;
   USB_HID_REPORT_T reports_data[1];
   ErrorCode_t ret = LPC_OK;

   /* Do a quick check of if the interface descriptor passed is the right one. */
   if( (pIntfDesc == 0) || (pIntfDesc->bInterfaceClass
       != USB_DEVICE_CLASS_HUMAN_INTERFACE)) {
      return ERR_FAILED;
   }

   /* Init HID params */
   memset((void *) &hid_param, 0, sizeof(USBD_HID_INIT_PARAM_T));
   hid_param.max_reports = 1;
   hid_param.mem_base = *mem_base;
   hid_param.mem_size = *mem_size;
   hid_param.intf_desc = (uint8_t *) pIntfDesc;

   /* user defined functions */
   hid_param.HID_GetReport = Keyboard_GetReport;
   hid_param.HID_SetReport = Keyboard_SetReport;
   hid_param.HID_EpIn_Hdlr  = Keyboard_EpIN_Hdlr;

   /* Init reports_data */
   reports_data[0].len = Keyboard_ReportDescSize;
   reports_data[0].idle_time = 0;
   reports_data[0].desc = (uint8_t *) &Keyboard_ReportDescriptor[0];
   hid_param.report_data  = reports_data;

   ret = USBD_API->hid->init(hUsb, &hid_param);

   /* update memory variables */
   *mem_base = hid_param.mem_base;
   *mem_size = hid_param.mem_size;

   /* store stack handle for later use. */
   g_keyBoard.hUsb = hUsb;
   g_keyBoard.tx_busy = 0;

   return ret;
}

/* Keyboard tasks */
void Keyboard_Tasks(void)
{
	/* check device is configured before sending report. */
	if ( USB_IsConfigured(g_keyBoard.hUsb)) {

		/* send report data */
		if (g_keyBoard.tx_busy == 0) {
			g_keyBoard.tx_busy = 1;

			/* update report based on board state */
			Keyboard_UpdateReport();
			USBD_API->hw->WriteEP( g_keyBoard.hUsb,
			                       HID_EP_IN,
			                       &g_keyBoard.report[0],
			                       KEYBOARD_REPORT_SIZE );
		}
	}
	else {
		/* reset busy flag if we get disconnected. */
		g_keyBoard.tx_busy = 0;
	}

}
