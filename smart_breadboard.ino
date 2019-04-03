/*

Copyright (c) Future Technology Devices International 2014

THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FTDI DRIVERS MAY BE USED ONLY IN CONJUNCTION WITH PRODUCTS BASED ON FTDI PARTS.

FTDI DRIVERS MAY BE DISTRIBUTED IN ANY FORM AS LONG AS LICENSE INFORMATION IS NOT MODIFIED.

IF A CUSTOM VENDOR ID AND/OR PRODUCT ID OR DESCRIPTION STRING ARE USED, IT IS THE
RESPONSIBILITY OF THE PRODUCT MANUFACTURER TO MAINTAIN ANY CHANGES AND SUBSEQUENT WHQL
RE-CERTIFICATION AS A RESULT OF MAKING THESE CHANGES.

Abstract:

Application to demonstrate gradient function of EVE.

Author : FTDI

Revision History:

*/

#include <stdio.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <avr/pgmspace.h>

#include "FT_Platform.h"

#define SAMAPP_DELAY_BTW_APIS (1000)
#define SAMAPP_ENABLE_DELAY() Ft_Gpu_Hal_Sleep(SAMAPP_DELAY_BTW_APIS)
#define SAMAPP_ENABLE_DELAY_VALUE(x) Ft_Gpu_Hal_Sleep(x)

/* Global variables for display resolution to support various display panels */
/* Default is WQVGA - 480x272 */
ft_int16_t FT_DispWidth = 480;
ft_int16_t FT_DispHeight = 272;
ft_int16_t FT_DispHCycle =  548;
ft_int16_t FT_DispHOffset = 43;
ft_int16_t FT_DispHSync0 = 0;
ft_int16_t FT_DispHSync1 = 41;
ft_int16_t FT_DispVCycle = 292;
ft_int16_t FT_DispVOffset = 12;
ft_int16_t FT_DispVSync0 = 0;
ft_int16_t FT_DispVSync1 = 10;
ft_uint8_t FT_DispPCLK = 5;
ft_char8_t FT_DispSwizzle = 0;
ft_char8_t FT_DispPCLKPol = 1;
ft_char8_t FT_DispCSpread = 1;
ft_char8_t FT_DispDither = 1;

#define ON          1
#define OFF         0
#define Font        27					// Font Size
#define MAX_LINES   4					// Max Lines allows to Display

/* Global used for buffer optimization */
Ft_Gpu_Hal_Context_t host,*phost;


ft_uint32_t Ft_CmdBuffer_Index;
ft_uint32_t Ft_DlBuffer_Index;

#ifdef BUFFER_OPTIMIZATION
ft_uint8_t  Ft_DlBuffer[FT_DL_SIZE];
ft_uint8_t  Ft_CmdBuffer[FT_CMD_FIFO_SIZE];
#endif
/* Boot up for FT800 followed by graphics primitive sample cases */
/* Initial boot up DL - make the back ground green color */
const ft_uint8_t FT_DLCODE_BOOTUP[12] =
{
  0,0,0,2,//GPU instruction CLEAR_COLOR_RGB
  7,0,0,38, //GPU instruction CLEAR
  0,0,0,0,  //GPU instruction DISPLAY
};
ft_void_t Ft_App_WrCoCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd)
{
  #ifdef  BUFFER_OPTIMIZATION
  /* Copy the command instruction into buffer */
  ft_uint32_t *pBuffcmd;
  pBuffcmd =(ft_uint32_t*)&Ft_CmdBuffer[Ft_CmdBuffer_Index];
  *pBuffcmd = cmd;
  #endif
  Ft_Gpu_Hal_WrCmd32(phost,cmd);
  /* Increment the command index */
  Ft_CmdBuffer_Index += FT_CMD_SIZE;
}

ft_void_t Ft_App_Flush_Co_Buffer(Ft_Gpu_Hal_Context_t *phost)
{
  #ifdef  BUFFER_OPTIMIZATION
  if (Ft_CmdBuffer_Index > 0)
  Ft_Gpu_Hal_WrCmdBuf(phost,Ft_CmdBuffer,Ft_CmdBuffer_Index);
  #endif
  Ft_CmdBuffer_Index = 0;
}


struct
{
  ft_uint8_t Key_Detect :1;
  ft_uint8_t Exit : 1;
}Flag;

static byte istouch()
{
  return !(Ft_Gpu_Hal_Rd16(phost,REG_TOUCH_RAW_XY) & 0x8000);
}
static ft_uint8_t sk=0;


ft_uint8_t Read_Keypad()
{
  static ft_uint8_t Read_tag=0,temp_tag=0,ret_tag=0,touch_detect=1;
  Read_tag = Ft_Gpu_Hal_Rd8(phost,REG_TOUCH_TAG);
  ret_tag = NULL;
  if(istouch()==0)  touch_detect = 0;
  if(Read_tag!=NULL)								// Allow if the Key is released
  {
    if(temp_tag!=Read_tag && touch_detect==0)
    {
      temp_tag = Read_tag;											// Load the Read tag to temp variable
      touch_detect = 1;
    }
  }
  else
  {
    if(temp_tag!=0)
    {
      Flag.Key_Detect = 1;
      Read_tag = temp_tag;
    }
    temp_tag = 0;
  }
  return Read_tag;
}

// Notepad buffer
void Notepad(void)
{
  /*local variables*/
  ft_uint8_t Line=0;
  ft_uint16_t Disp_pos = 0,But_opt;
  ft_uint8_t Read_sfk=0,	tval;
  ft_uint16_t noofchars=0,line2disp =0,nextline = 0;
  ft_uint8_t   font = 27;
  ft_uint8_t button_active = 0;

  char *state_string = malloc(10 * sizeof(char));
  state_string[9] = '\0';
  const ft_uint16_t NEXT_SFK = 201;
  const ft_uint16_t PREV_SFK = 200;
  const ft_uint16_t NUM_STATES = 30;
  const ft_uint16_t NUM_PINS = 30;
  const ft_uint16_t PIN_OFFSET = 22;
  ft_uint16_t current_state = 0;
  ft_uint32_t state_outputs[NUM_STATES] = {
    0x00000001,
    0x00000002,
    0x00000004,
    0x00000008,
    0x00000010,
    0x00000020,
    0x00000040,
    0x00000080,
    0x00000100,
    0x00000200,
    0x00000400,
    0x00000800,
    0x00001000,
    0x00002000,
    0x00004000,
    0x00008000,
    0x00010000,
    0x00020000,
    0x00040000,
    0x00080000,
    0x00100000,
    0x00200000,
    0x00400000,
    0x00800000,
    0x01000000,
    0x02000000,
    0x04000000,
    0x08000000,
    0x10000000,
    0x20000000,
  };

  sprintf(state_string, "%d", current_state + PIN_OFFSET);

  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(PIN_OFFSET + i, OUTPUT);
  }

  /*enter*/
  Flag.Exit = 0;
  do
  {
    Read_sfk = Read_Keypad();                // read the keys


    // Start the new Display list
    Ft_Gpu_CoCmd_Dlstart(phost);
    Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(100,100,100));
    Ft_App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
    Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));
    Ft_App_WrCoCmd_Buffer(phost,TAG_MASK(1));            // enable tagbuffer updation
    Ft_Gpu_CoCmd_FgColor(phost,0x663300);
    Ft_Gpu_CoCmd_BgColor(phost,0x662244);

    if (Read_sfk == NEXT_SFK && current_state + 1 < NUM_STATES && !button_active) {
      current_state++;
      sprintf(state_string, "%d", current_state + PIN_OFFSET);
      button_active = 1;
    } else if (Read_sfk == PREV_SFK && current_state > 0 && !button_active) {
      current_state--;
      sprintf(state_string, "%d", current_state + PIN_OFFSET);
      button_active = 1;
    } else if (Read_sfk != NEXT_SFK && Read_sfk != PREV_SFK){
      button_active = 0;
    }

    ft_uint32_t temp = state_outputs[current_state];
    for (int pin = 0; pin < NUM_PINS; pin++) {
      if (temp & 0x01) {
        digitalWrite(PIN_OFFSET + pin, HIGH);
      } else {
        digitalWrite(PIN_OFFSET + pin, LOW);
      }

      temp >>= 1;
    }

    int test;
    if (Read_sfk == NEXT_SFK) {
      test = 150;
    } else {
      test = 100;
    }
    But_opt = (Read_sfk== NEXT_SFK)?  OPT_FLAT:0;
    Ft_App_WrCoCmd_Buffer(phost,TAG(NEXT_SFK));
    Ft_Gpu_CoCmd_Button(phost,(FT_DispWidth - 100),(FT_DispHeight*0.01),98,(FT_DispHeight*0.98),28,But_opt,"Next");
    But_opt = (Read_sfk== PREV_SFK)?  OPT_FLAT:0;
    Ft_App_WrCoCmd_Buffer(phost,TAG(PREV_SFK));
    Ft_Gpu_CoCmd_Button(phost,2,(FT_DispHeight*0.01),98,(FT_DispHeight*0.98),28,But_opt,"Prev");
    Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2,FT_DispHeight/2 + 15,26,OPT_CENTERX|OPT_CENTERY,state_string);
    Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
    Ft_Gpu_CoCmd_Swap(phost);
    Ft_App_Flush_Co_Buffer(phost);
    Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
  }while(1);

}
/***********************API used to SET the ICON******************************************/
/*Refer the code flow in the flowchart availble in the Application Note */
/* Information API*/
ft_void_t Info()
{
  ft_uint16_t dloffset = 0,z;
  Ft_CmdBuffer_Index = 0;
  // Touch Screen Calibration
  Ft_Gpu_CoCmd_Dlstart(phost);
  Ft_App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
  Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));
  Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2,FT_DispHeight/2 - 15,26,OPT_CENTERX|OPT_CENTERY,"Touch Screen Calibration");
  Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2,FT_DispHeight/2 + 15,26,OPT_CENTERX|OPT_CENTERY,"Touch the dots");
  Ft_Gpu_CoCmd_Calibrate(phost,0);
  Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
  Ft_Gpu_CoCmd_Swap(phost);
  Ft_App_Flush_Co_Buffer(phost);
  Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
}
ft_void_t Ft_BootupConfig()
{
  Ft_Gpu_Hal_Powercycle(phost,FT_TRUE);

  /* Access address 0 to wake up the FT800 */
  Ft_Gpu_HostCommand(phost,FT_GPU_ACTIVE_M);
  Ft_Gpu_Hal_Sleep(20);

  /* Set the clk to external clock */
  #ifndef ME800A_HV35R
  Ft_Gpu_HostCommand(phost,FT_GPU_EXTERNAL_OSC);
  Ft_Gpu_Hal_Sleep(10);
  #endif

  {
    ft_uint8_t chipid;
    //Read Register ID to check if FT800 is ready.
    chipid = Ft_Gpu_Hal_Rd8(phost, REG_ID);
    while(chipid != 0x7C)
    {
      chipid = Ft_Gpu_Hal_Rd8(phost, REG_ID);
      printf("VC1 register ID after wake up %x\n",chipid);
      ft_delay(100);
    }
    #if defined(MSVC_PLATFORM) || defined (FT900_PLATFORM)
    printf("VC1 register ID after wake up %x\n",chipid);
    #endif
  }

  Ft_Gpu_Hal_Wr16(phost, REG_HCYCLE, FT_DispHCycle);
  Ft_Gpu_Hal_Wr16(phost, REG_HOFFSET, FT_DispHOffset);
  Ft_Gpu_Hal_Wr16(phost, REG_HSYNC0, FT_DispHSync0);
  Ft_Gpu_Hal_Wr16(phost, REG_HSYNC1, FT_DispHSync1);
  Ft_Gpu_Hal_Wr16(phost, REG_VCYCLE, FT_DispVCycle);
  Ft_Gpu_Hal_Wr16(phost, REG_VOFFSET, FT_DispVOffset);
  Ft_Gpu_Hal_Wr16(phost, REG_VSYNC0, FT_DispVSync0);
  Ft_Gpu_Hal_Wr16(phost, REG_VSYNC1, FT_DispVSync1);
  Ft_Gpu_Hal_Wr8(phost, REG_SWIZZLE, FT_DispSwizzle);
  Ft_Gpu_Hal_Wr8(phost, REG_PCLK_POL, FT_DispPCLKPol);
  Ft_Gpu_Hal_Wr16(phost, REG_HSIZE, FT_DispWidth);
  Ft_Gpu_Hal_Wr16(phost, REG_VSIZE, FT_DispHeight);
  Ft_Gpu_Hal_Wr16(phost, REG_CSPREAD, FT_DispCSpread);
  Ft_Gpu_Hal_Wr16(phost, REG_DITHER, FT_DispDither);

  #if (defined(ENABLE_FT_800) || defined(ENABLE_FT_810) ||defined(ENABLE_FT_812))
  /* Touch configuration - configure the resistance value to 1200 - this value is specific to customer requirement and derived by experiment */
  Ft_Gpu_Hal_Wr16(phost, REG_TOUCH_RZTHRESH,1200);
  #endif
  Ft_Gpu_Hal_Wr8(phost, REG_GPIO_DIR,0xff);
  Ft_Gpu_Hal_Wr8(phost, REG_GPIO,0xff);


  /*It is optional to clear the screen here*/
  Ft_Gpu_Hal_WrMem(phost, RAM_DL,(ft_uint8_t *)FT_DLCODE_BOOTUP,sizeof(FT_DLCODE_BOOTUP));
  Ft_Gpu_Hal_Wr8(phost, REG_DLSWAP,DLSWAP_FRAME);


  Ft_Gpu_Hal_Wr8(phost, REG_PCLK,FT_DispPCLK);//after this display is visible on the LCD




  /* make the spi to quad mode - addition 2 bytes for silicon */
  #ifdef FT_81X_ENABLE
  /* api to set quad and numbe of dummy bytes */
  #ifdef ENABLE_SPI_QUAD
  Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_QUAD_CHANNEL,FT_GPU_SPI_TWODUMMY);
  #elif ENABLE_SPI_DUAL
  Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_QUAD_CHANNEL,FT_GPU_SPI_TWODUMMY);
  #else
  Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_SINGLE_CHANNEL,FT_GPU_SPI_ONEDUMMY);

  #endif
  #endif
  phost->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(phost,REG_CMD_WRITE);

}


/* Main entry point */
ft_void_t setup() {

  ft_uint8_t chipid;
  Ft_Gpu_HalInit_t halinit;
  halinit.TotalChannelNum = 1;
  Ft_Gpu_Hal_Init(&halinit);
  host.hal_config.channel_no = 0;
  host.hal_config.pdn_pin_no = FT800_PD_N;
  host.hal_config.spi_cs_pin_no = FT800_SEL_PIN;
  host.hal_config.spi_clockrate_khz = 4000; //in KHz

  Ft_Gpu_Hal_Open(&host);
  phost = &host;

  Ft_BootupConfig();
  Info(); //Set screen calibration
  Ft_Gpu_Hal_Sleep(100);
  Notepad(); // Main loop
}

void loop()
{
}


/* Nothing beyond this */
