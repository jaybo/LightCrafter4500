/*
 * API.cpp
 *
 * This module provides C callable APIs for each of the command supported by LightCrafter4500 platform and detailed in the programmer's guide.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#include "API.h"
#include "string.h"
#include "usb.h"
#include "Common.h"
#include <stdlib.h>

extern unsigned char OutputBuffer[];
extern unsigned char InputBuffer[];

CmdFormat CmdList[255] =
{
    {   0x1A,  0x00,  0x01   },      //SOURCE_SEL,
    {   0x1A,  0x02,  0x01   },      //PIXEL_FORMAT,
    {   0x1A,  0x03,  0x01   },      //CLK_SEL,
    {   0x1A,  0x37,  0x01   },      //CHANNEL_SWAP,
    {   0x1A,  0x04,  0x01   },      //FPD_MODE,
    {   0,  0,  0   },      //CURTAIN_COLOR,
    {   0x02,  0x00,  0x01   },      //POWER_CONTROL,
    {   0x10,  0x08,  0x01   },      //FLIP_LONG,
    {   0x10,  0x09,  0x01   },      //FLIP_SHORT,
    {   0x12,  0x03,  0x01   },      //TPG_SEL,
    {   0x1A,  0x05,  0x01   },      //PWM_INVERT,
    {   0x1A,  0x07,  0x01   },      //LED_ENABLE,
    {   0x02,  0x05,  0x00   },      //GET_VERSION,
    {   0x08,  0x02,  0x00   },      //SW_RESET,
    {   0,  0,  0   },      //DMD_PARK,
    {   0,  0,  0   },      //BUFFER_FREEZE,
    {   0x1A,  0x0A,  0x00   },      //STATUS_HW,
    {   0x1A,  0x0B,  0x00   },      //STATUS_SYS,
    {   0x1A,  0x0C,  0x00   },      //STATUS_MAIN,
    {   0,  0,  0   },      //CSC_DATA,
    {   0,  0,  0   },      //GAMMA_CTL,
    {   0,  0,  0   },      //BC_CTL,
    {   0x1A,  0x10,  0x01   },      //PWM_ENABLE,
    {   0x1A,  0x11,  0x06   },      //PWM_SETUP,
    {   0x1A,  0x12,  0x05   },      //PWM_CAPTURE_CONFIG,
    {   0x1A,  0x38,  0x02   },      //GPIO_CONFIG,
    {   0x0B,  0x01,  0x03   },      //LED_CURRENT,
    {   0x10,  0x00,  0x10   },      //DISP_CONFIG,
    {   0,  0,  0   },      //TEMP_CONFIG,
    {   0,  0,  0   },      //TEMP_READ,
    {   0x1A,  0x16,  0x09   },      //MEM_CONTROL,
    {   0,  0,  0   },      //I2C_CONTROL,
    {   0x1A,  0x1A,  0x01   },      //LUT_VALID,
    {   0x1A,  0x1B,  0x01   },      //DISP_MODE,
    {   0x1A,  0x1D,  0x03   },      //TRIG_OUT1_CTL,
    {   0x1A,  0x1E,  0x03   },      //TRIG_OUT2_CTL,
    {   0x1A,  0x1F,  0x02   },      //RED_STROBE_DLY,
    {   0x1A,  0x20,  0x02   },      //GRN_STROBE_DLY,
    {   0x1A,  0x21,  0x02   },      //BLU_STROBE_DLY,
    {   0x1A,  0x22,  0x01   },      //PAT_DISP_MODE,
    {   0x1A,  0x23,  0x01   },      //PAT_TRIG_MODE,
    {   0x1A,  0x24,  0x01   },      //PAT_START_STOP,
    {   0,  0,  0   },      //BUFFER_SWAP,
    {   0,  0,  0   },      //BUFFER_WR_DISABLE,
    {   0,  0,  0   },      //CURRENT_RD_BUFFER,
    {   0x1A,  0x29,  0x08   },      //PAT_EXPO_PRD,
    {   0x1A,  0x30,  0x01   },      //INVERT_DATA,
    {   0x1A,  0x31,  0x04   },      //PAT_CONFIG,
    {   0x1A,  0x32,  0x01   },      //MBOX_ADDRESS,
    {   0x1A,  0x33,  0x01   },      //MBOX_CONTROL,
    {   0x1A,  0x34,  0x00   },      //MBOX_DATA,
    {   0x1A,  0x35,  0x04   },      //TRIG_IN1_DELAY,
    {   0,  0,  0   },      //TRIG_IN2_CONTROL,
    {   0x1A,  0x39,  0x01   },      //SPLASH_LOAD,
    {   0x1A,  0x3A,  0x02   },      //SPLASH_LOAD_TIMING,
    {   0x08,  0x07,  0x03   },      //GPCLK_CONFIG,
    {   0,  0,  0   },      //PULSE_GPIO_23,
    {   0,  0,  0   },      //ENABLE_LCR_DEBUG,
    {   0x12,  0x04,  0x0C   },      //TPG_COLOR,
    {   0x1A,  0x13,  0x05   },     //PWM_CAPTURE_READ,
    {   0x30,  0x01,  0x00   },     //PROG_MODE,
    {   0x00,  0x00,  0x00   },     //BL_STATUS
    {   0x00,  0x23,  0x01   },     //BL_SPL_MODE
    {   0x00,  0x15,  0x01   },     //BL_GET_MANID,
    {   0x00,  0x15,  0x01   },     //BL_GET_DEVID,
    {   0x00,  0x15,  0x01   },     //BL_GET_CHKSUM,
    {   0x00,  0x29,  0x04   },     //BL_SETSECTADDR,
    {   0x00,  0x28,  0x00   },     //BL_SECT_ERASE,
    {   0x00,  0x2C,  0x04   },     //BL_SET_DNLDSIZE,
    {   0x00,  0x25,  0x00   },     //BL_DNLD_DATA,
    {   0x00,  0x2F,  0x01   },     //BL_FLASH_TYPE,
    {   0x00,  0x26,  0x00   },     //BL_CALC_CHKSUM,
    {   0x00,  0x30,  0x01   }     //BL_PROG_MODE,
};

static unsigned char seqNum=0;
static unsigned int PatLut[128] = {0};
static unsigned int PatLutIndex = 0;

int LCR_Write()
{
    return USB_Write();
}

int LCR_Read()
/**
 * This function is private to this file. This function is called to write the read control command and then read back 64 bytes over USB
 * to InputBuffer.
 *
 * @return  number of bytes read
 *          -2 = nack from target
 *          -1 = error reading
 *
 */
{
    int ret_val;
    hidMessageStruct *pMsg = (hidMessageStruct *)InputBuffer;
    if(USB_Write() > 0)
    {
        ret_val =  USB_Read();

        if((pMsg->head.flags.nack == 1) || (pMsg->head.length == 0))
            return -2;
        else
            return ret_val;
    }
    return -1;
}

int LCR_ContinueRead()
{
    return USB_Read();
}

int LCR_SendMsg(hidMessageStruct *pMsg)
/**
 * This function is private to this file. This function is called to send a message over USB; in chunks of 64 bytes.
 *
 * @return  number of bytes sent
 *          -1 = FAIL
 *
 */
{
    int maxDataSize = USB_MAX_PACKET_SIZE-sizeof(pMsg->head);
    int dataBytesSent = MIN(pMsg->head.length, maxDataSize);    //Send all data or max possible

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], pMsg, (sizeof(pMsg->head) + dataBytesSent));

    if(LCR_Write() < 0)
        return -1;

    //dataBytesSent = maxDataSize;

    while(dataBytesSent < pMsg->head.length)
    {
        memcpy(&OutputBuffer[1], &pMsg->text.data[dataBytesSent], USB_MAX_PACKET_SIZE);
        if(LCR_Write() < 0)
            return -1;
        dataBytesSent += USB_MAX_PACKET_SIZE;
    }
    return dataBytesSent+sizeof(pMsg->head);
}

int LCR_PrepReadCmd(LCR_CMD cmd)
/**
 * This function is private to this file. Prepares the read-control command packet for the given command code and copies it to OutputBuffer.
 *
 * @param   cmd  - I - USB command code.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    hidMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.nack = 0;
    msg.head.seq = 0;

    msg.text.cmd = (CmdList[cmd].CMD2 << 8) | CmdList[cmd].CMD3;
    msg.head.length = 2;

    if(cmd == BL_GET_MANID)
    {
        msg.text.data[2] = 0x0C;
        msg.head.length += 1;
    }
    else if (cmd == BL_GET_DEVID)
    {
        msg.text.data[2] = 0x0D;
        msg.head.length += 1;
    }
    else if (cmd == BL_GET_CHKSUM)
    {
        msg.text.data[2] = 0x00;
        msg.head.length += 1;
    }

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.text.cmd) + msg.head.length));
    return 0;
}

int LCR_PrepReadCmdWithParam(LCR_CMD cmd, unsigned char param)
/**
 * This function is private to this file. Prepares the read-control command packet for the given command code and parameter and copies it to OutputBuffer.
 *
 * @param   cmd  - I - USB command code.
 * @param   param - I - parameter to be used for tis read command.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    hidMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.nack = 0;
    msg.head.seq = 0;

    msg.text.cmd = (CmdList[cmd].CMD2 << 8) | CmdList[cmd].CMD3;
    msg.head.length = 3;

    msg.text.data[2] = param;

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.text.cmd) + msg.head.length));
    return 0;
}

int LCR_PrepMemReadCmd(unsigned int addr)
/**
 * This function is private to this file. Prepares the memory read command packet with the given address and copies it to OutputBuffer.
 *
 * @param   addr  - I - memory address in controller to be read.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    hidMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.nack = 0;
    msg.head.seq = 0;

    msg.text.cmd = (CmdList[MEM_CONTROL].CMD2 << 8) | CmdList[MEM_CONTROL].CMD3;
    msg.head.length = 6;

    msg.text.data[2] = addr;
    msg.text.data[3] = addr >>8;
    msg.text.data[4] = addr >>16;
    msg.text.data[5] = addr >>24;

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.text.cmd) + msg.head.length));
    return 0;
}

int LCR_PrepWriteCmd(hidMessageStruct *pMsg, LCR_CMD cmd)
/**
 * This function is private to this file. Prepares the write command packet with given command code in the message structure pointer passed.
 *
 * @param   cmd  - I - USB command code.
 * @param   pMsg - I - Pointer to the message.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    pMsg->head.flags.rw = 0; //Write
    pMsg->head.flags.reply = 0; //Host wants a reply from device
    pMsg->head.flags.dest = 0; //Projector Control Endpoint
    pMsg->head.flags.reserved = 0;
    pMsg->head.flags.nack = 0;
    pMsg->head.seq = seqNum++;

    pMsg->text.cmd = (CmdList[cmd].CMD2 << 8) | CmdList[cmd].CMD3;
    pMsg->head.length = CmdList[cmd].len + 2;

    return 0;
}

int LCR_GetVersion(unsigned int *pApp_ver, unsigned int *pAPI_ver, unsigned int *pSWConfig_ver, unsigned int *pSeqConfig_ver)
/**
 * This command reads the version information of the DLPC350 firmware.
 * (I2C: 0x11)
 * (USB: CMD2: 0x02, CMD3: 0x05)
 *
 * @param   pApp_ver  - O - Application Software Revision BITS 0:15 PATCH NUMBER, BITS 16:23 MINOR REVISION, BIS 24:31 MAJOR REVISION
 * @param   pAPI_ver  - O - API Software Revision BITS 0:15 PATCH NUMBER, BITS 16:23 MINOR REVISION, BIS 24:31 MAJOR REVISION
 * @param   pSWConfig_ver  - O - Software Configuration Revision BITS 0:15 PATCH NUMBER, BITS 16:23 MINOR REVISION, BIS 24:31 MAJOR REVISION
 * @param   pSeqConfig_ver  - O - Sequence Configuration Revision BITS 0:15 PATCH NUMBER, BITS 16:23 MINOR REVISION, BIS 24:31 MAJOR REVISION
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(GET_VERSION);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pApp_ver = *(unsigned int *)&msg.text.data[0];
        *pAPI_ver = *(unsigned int *)&msg.text.data[4];
        *pSWConfig_ver = *(unsigned int *)&msg.text.data[8];
        *pSeqConfig_ver = *(unsigned int *)&msg.text.data[12];
        return 0;
    }
    return -1;
}

int LCR_GetLedEnables(bool *pSeqCtrl, bool *pRed, bool *pGreen, bool *pBlue)
/**
 * This command reads back the state of LED control method as well as the enabled/disabled status of all LEDs.
 * (I2C: 0x10)
 * (USB: CMD2: 0x1A, CMD3: 0x07)
 *
 * @param   pSeqCtrl  - O - 1 - All LED enables are controlled by the Sequencer and ignore the other LED enable settings.
 *                          0 - All LED enables are controlled by pRed, pGreen and pBlue seetings and ignore Sequencer control
 * @param   pRed  - O - 0 - Red LED is disabled
 *                      1 - Red LED is enabled
 * @param   pGreen  - O - 0 - Green LED is disabled
 *                      1 - Green LED is enabled
 * @param   pBlue  - O - 0 - Blue LED is disabled
 *                      1 - Blue LED is enabled]
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(LED_ENABLE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if(msg.text.data[0] & BIT0)
            *pRed = true;
        else
            *pRed = false;

        if(msg.text.data[0] & BIT1)
            *pGreen = true;
        else
            *pGreen = false;

        if(msg.text.data[0] & BIT2)
            *pBlue = true;
        else
            *pBlue = false;

        if(msg.text.data[0] & BIT3)
            *pSeqCtrl = true;
        else
            *pSeqCtrl = false;
        return 0;
    }
    return -1;
}


int LCR_SetLedEnables(bool SeqCtrl, bool Red, bool Green, bool Blue)
/**
 * This command sets the state of LED control method as well as the enabled/disabled status of all LEDs.
 * (I2C: 0x10)
 * (USB: CMD2: 0x1A, CMD3: 0x07)
 *
 * @param   pSeqCtrl  - I - 1 - All LED enables are controlled by the Sequencer and ignore the other LED enable settings.
 *                          0 - All LED enables are controlled by pRed, pGreen and pBlue seetings and ignore Sequencer control
 * @param   pRed  - I - 0 - Red LED is disabled
 *                      1 - Red LED is enabled
 * @param   pGreen  - I - 0 - Green LED is disabled
 *                      1 - Green LED is enabled
 * @param   pBlue  - I - 0 - Blue LED is disabled
 *                      1 - Blue LED is enabled]
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;
    unsigned char Enable=0;

    if(SeqCtrl)
        Enable |= BIT3;
    if(Red)
        Enable |= BIT0;
    if(Green)
        Enable |= BIT1;
    if(Blue)
        Enable |= BIT2;

    msg.text.data[2] = Enable;
    LCR_PrepWriteCmd(&msg, LED_ENABLE);

    return LCR_SendMsg(&msg);
}

int LCR_GetLedCurrents(unsigned char *pRed, unsigned char *pGreen, unsigned char *pBlue)
/**
 * (I2C: 0x4B)
 * (USB: CMD2: 0x0B, CMD3: 0x01)
 * This parameter controls the pulse duration of the specific LED PWM modulation output pin. The resolution
 * is 8 bits and corresponds to a percentage of the LED current. The PWM value can be set from 0 to 100%
 * in 256 steps . If the LED PWM polarity is set to normal polarity, a setting of 0xFF gives the maximum
 * PWM current. The LED current is a function of the specific LED driver design.
 *
 * @param   pRed  - O - Red LED PWM current control Valid range, assuming normal polarity of PWM signals, is:
 *                      0x00 (0% duty cycle → Red LED driver generates no current
 *                      0xFF (100% duty cycle → Red LED driver generates maximum current))
 *                      The current level corresponding to the selected PWM duty cycle is a function of the specific LED driver design and thus varies by design.
 * @param   pGreen  - O - Green LED PWM current control Valid range, assuming normal polarity of PWM signals, is:
 *                      0x00 (0% duty cycle → Red LED driver generates no current
 *                      0xFF (100% duty cycle → Red LED driver generates maximum current))
 *                      The current level corresponding to the selected PWM duty cycle is a function of the specific LED driver design and thus varies by design.
 * @param   pBlue  - O - Blue LED PWM current control Valid range, assuming normal polarity of PWM signals, is:
 *                      0x00 (0% duty cycle → Red LED driver generates no current
 *                      0xFF (100% duty cycle → Red LED driver generates maximum current))
 *                      The current level corresponding to the selected PWM duty cycle is a function of the specific LED driver design and thus varies by design.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(LED_CURRENT);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pRed = msg.text.data[0];
        *pGreen = msg.text.data[1];
        *pBlue = msg.text.data[2];

        return 0;
    }
    return -1;
}


int LCR_SetLedCurrents(unsigned char RedCurrent, unsigned char GreenCurrent, unsigned char BlueCurrent)
/**
 * (I2C: 0x4B)
 * (USB: CMD2: 0x0B, CMD3: 0x01)
 * This parameter controls the pulse duration of the specific LED PWM modulation output pin. The resolution
 * is 8 bits and corresponds to a percentage of the LED current. The PWM value can be set from 0 to 100%
 * in 256 steps . If the LED PWM polarity is set to normal polarity, a setting of 0xFF gives the maximum
 * PWM current. The LED current is a function of the specific LED driver design.
 *
 * @param   RedCurrent  - I - Red LED PWM current control Valid range, assuming normal polarity of PWM signals, is:
 *                      0x00 (0% duty cycle → Red LED driver generates no current
 *                      0xFF (100% duty cycle → Red LED driver generates maximum current))
 *                      The current level corresponding to the selected PWM duty cycle is a function of the specific LED driver design and thus varies by design.
 * @param   GreenCurrent  - I - Green LED PWM current control Valid range, assuming normal polarity of PWM signals, is:
 *                      0x00 (0% duty cycle → Red LED driver generates no current
 *                      0xFF (100% duty cycle → Red LED driver generates maximum current))
 *                      The current level corresponding to the selected PWM duty cycle is a function of the specific LED driver design and thus varies by design.
 * @param   BlueCurrent  - I - Blue LED PWM current control Valid range, assuming normal polarity of PWM signals, is:
 *                      0x00 (0% duty cycle → Red LED driver generates no current
 *                      0xFF (100% duty cycle → Red LED driver generates maximum current))
 *                      The current level corresponding to the selected PWM duty cycle is a function of the specific LED driver design and thus varies by design.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = RedCurrent;
    msg.text.data[3] = GreenCurrent;
    msg.text.data[4] = BlueCurrent;

    LCR_PrepWriteCmd(&msg, LED_CURRENT);

    return LCR_SendMsg(&msg);
}

bool LCR_GetLongAxisImageFlip(void)
/**
 * (I2C: 0x08)
 * (USB: CMD2: 0x10, CMD3: 0x08)
 * The Long-Axis Image Flip defines whether the input image is flipped across the long axis of the DMD. If
 * this parameter is changed while displaying a still image, the input still image should be re-sent. If the
 * image is not re-sent, the output image might be slightly corrupted. In Structured Light mode, the image
 * flip will take effect on the next bit-plane, image, or video frame load.
 *
 * @return  TRUE = Image flipped along long axis    <BR>
 *          FALSE = Image not flipped  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(FLIP_LONG);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if ((msg.text.data[0] & BIT0) == BIT0)
            return true;
        else
            return false;
    }
    return false;
}

bool LCR_GetShortAxisImageFlip(void)
/**
 * (I2C: 0x09)
 * (USB: CMD2: 0x10, CMD3: 0x09)
 * The Short-Axis Image Flip defines whether the input image is flipped across the short axis of the DMD. If
 * this parameter is changed while displaying a still image, the input still image should be re-sent. If the
 * image is not re-sent, the output image might be slightly corrupted. In Structured Light mode, the image
 * flip will take effect on the next bit-plane, image, or video frame load.
 *
 * @return  TRUE = Image flipped along short axis    <BR>
 *          FALSE = Image not flipped  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(FLIP_SHORT);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        if ((msg.text.data[0] & BIT0) == BIT0)
            return true;
        else
            return false;
    }
    return false;
}


int LCR_SetLongAxisImageFlip(bool Flip)
/**
 * (I2C: 0x08)
 * (USB: CMD2: 0x10, CMD3: 0x08)
 * The Long-Axis Image Flip defines whether the input image is flipped across the long axis of the DMD. If
 * this parameter is changed while displaying a still image, the input still image should be re-sent. If the
 * image is not re-sent, the output image might be slightly corrupted. In Structured Light mode, the image
 * flip will take effect on the next bit-plane, image, or video frame load.
 *
 * @param   Flip -I TRUE = Image flipped along long axis enable    <BR>
 *                  FALSE = Do not flip image <BR>
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    if(Flip)
        msg.text.data[2] = BIT0;
    else
        msg.text.data[2] = 0;

    LCR_PrepWriteCmd(&msg, FLIP_LONG);

    return LCR_SendMsg(&msg);
}

int LCR_SetShortAxisImageFlip(bool Flip)
/**
 * (I2C: 0x09)
 * (USB: CMD2: 0x10, CMD3: 0x09)
 * The Long-Axis Image Flip defines whether the input image is flipped across the long axis of the DMD. If
 * this parameter is changed while displaying a still image, the input still image should be re-sent. If the
 * image is not re-sent, the output image might be slightly corrupted. In Structured Light mode, the image
 * flip will take effect on the next bit-plane, image, or video frame load.
 *
 * @param   Flip -I TRUE = Image flipped along long axis enable    <BR>
 *                  FALSE = Do not flip image <BR>
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    if(Flip)
        msg.text.data[2] = BIT0;
    else
        msg.text.data[2] = 0;

    LCR_PrepWriteCmd(&msg, FLIP_SHORT);

    return LCR_SendMsg(&msg);
}

int LCR_EnterProgrammingMode()
/**
 * This function is to be called to put the unit in programming mode. Only programming mode APIs will work once
 * in this mode.
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = 1;

    LCR_PrepWriteCmd(&msg, PROG_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_ExitProgrammingMode(void)
/**
 * This function works only in prorgamming mode.
 * This function is to be called to exit programming mode and resume normal operation with the new downloaded firmware.
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = 2;
    LCR_PrepWriteCmd(&msg, BL_PROG_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetFlashManID(unsigned short *pManID)
/**
 * This function works only in prorgamming mode.
 * This function returns the manufacturer ID of the flash part interfaced with the controller.
 *
 * @param pManID - O - Manufacturer ID of the flash part
 *
 * @return 0 PASS <BR>
 *         -1 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(BL_GET_MANID);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pManID = msg.text.data[6];
        *pManID |= (unsigned short)msg.text.data[7] << 8;
        return 0;
    }
    return -1;
}

int LCR_GetFlashDevID(unsigned long long *pDevID)
/**
 * This function works only in prorgamming mode.
 * This function returns the device ID of the flash part interfaced with the controller.
 *
 * @param pDevID - O - Device ID of the flash part
 *
 * @return 0 PASS <BR>
 *         -1 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(BL_GET_DEVID);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pDevID = msg.text.data[6];
        *pDevID |= (unsigned long long)msg.text.data[7] << 8;
        *pDevID |= (unsigned long long)msg.text.data[8] << 16;
        *pDevID |= (unsigned long long)msg.text.data[9] << 24;
        *pDevID |= (unsigned long long)msg.text.data[12] << 32;
        *pDevID |= (unsigned long long)msg.text.data[13] << 40;
        *pDevID |= (unsigned long long)msg.text.data[14] << 48;
        *pDevID |= (unsigned long long)msg.text.data[15] << 56;
        return 0;
    }
    return -1;
}

int LCR_GetBLStatus(unsigned char *BL_Status)
/**
 * This function works only in prorgamming mode.
 * This function returns the device ID of the flash part interfaced with the controller.
 *
 * @param BL_Status - O - BIT3 of the status byte when set indicates that the program is busy
 *                        with exectuing the previous command. When BIT3 is reset, it means the
 *                        program is ready for the next command.
 *
 * @return 0 PASS <BR>
 *         -1 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    /* For some reason BL_STATUS readback is not working properly.
     * However, after going through the bootloader code, I have ascertained that any
     * readback is fine - Byte 0 is always the bootloader status */
    LCR_PrepReadCmd(BL_GET_CHKSUM);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *BL_Status = msg.text.data[0];
        return 0;
    }
    return -1;
}

int LCR_SetFlashAddr(unsigned int Addr)
/**
 * This function works only in prorgamming mode.
 * This function is to be called to set the address prior to calling LCR_FlashSectorErase or LCR_DownloadData APIs.
 *
 * @param Addr - I - 32-bit absolute address.
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = Addr;
    msg.text.data[3] = Addr >> 8;
    msg.text.data[4] = Addr >> 16;
    msg.text.data[5] = Addr >> 24;

    LCR_PrepWriteCmd(&msg, BL_SET_SECTADDR);

    return LCR_SendMsg(&msg);
}

 int LCR_FlashSectorErase(void)
 /**
  * This function works only in prorgamming mode.
  * This function is to be called to erase a sector of flash. The address of the sector to be erased
  * is to be set by using the LCR_SetFlashAddr() API
  *
  * @return >=0 PASS <BR>
  *         <0 FAIL <BR>
  *
  */
 {
     hidMessageStruct msg;

     LCR_PrepWriteCmd(&msg, BL_SECT_ERASE);
     return LCR_SendMsg(&msg);
 }

int LCR_SetDownloadSize(unsigned int dataLen)
/**
 * This function works only in prorgamming mode.
 * This function is to be called to set the payload size of data to be sent using LCR_DownloadData API.
 *
 * @param dataLen -I - length of download data payload in bytes.
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = dataLen;
    msg.text.data[3] = dataLen >> 8;
    msg.text.data[4] = dataLen >> 16;
    msg.text.data[5] = dataLen >> 24;

    LCR_PrepWriteCmd(&msg, BL_SET_DNLDSIZE);

    return LCR_SendMsg(&msg);
}

int LCR_DownloadData(unsigned char *pByteArray, unsigned int dataLen)
/**
 * This function works only in prorgamming mode.
 * This function sends one payload of data to the controller at a time. takes the total size of payload
 * in the parameter dataLen and returns the actual number of bytes that was sent in the return value.
 * This function needs to be called multiple times until all of the desired bytes are sent.
 *
 * @param pByteArray - I - Pointer to where the data to be downloaded is to be fetched from
 * @param dataLen -I - length in bytes of the total payload data to download.
 *
 * @return number of bytes actually downloaded <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;
    int retval;
    unsigned int sendSize;

    sendSize = HID_MESSAGE_MAX_SIZE - sizeof(msg.head)- sizeof(msg.text.cmd) - 2;//The last -2 is to workaround a bug in bootloader.

    if(dataLen > sendSize)
        dataLen = sendSize;

    CmdList[BL_DNLD_DATA].len = dataLen;
    memcpy(&msg.text.data[2], pByteArray, dataLen);

    LCR_PrepWriteCmd(&msg, BL_DNLD_DATA);

    retval = LCR_SendMsg(&msg);
    if(retval > 0)
        return dataLen;

    return -1;
}

void LCR_WaitForFlashReady()
/**
 * This function works only in prorgamming mode.
 * This function polls the status bit and returns only when the controller is ready for next command.
 *
 */
{
    unsigned char BLstatus=STAT_BIT_FLASH_BUSY;

    do
    {
        LCR_GetBLStatus(&BLstatus);
    }
    while((BLstatus & STAT_BIT_FLASH_BUSY) == STAT_BIT_FLASH_BUSY);//Wait for flash busy flag to go off
}

int LCR_SetFlashType(unsigned char Type)
/**
 * This function works only in prorgamming mode.
 * This function is to be used to set the programming type of the flash device attached to the controller.
 *
 * @param Type - I - Type of the flash device.
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = Type;

    LCR_PrepWriteCmd(&msg, BL_FLASH_TYPE);

    return LCR_SendMsg(&msg);
}

int LCR_CalculateFlashChecksum(void)
/**
 * This function works only in prorgamming mode.
 * This function is to be issued to instruct the controller to calculate the flash checksum.
 * LCR_WaitForFlashReady() is then to be called to ensure that the controller is done and then call
 * LCR_GetFlashChecksum() API to retrieve the actual checksum from the controller.
 *
 * @return 0 = PASS <BR>
 *         -1 = FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepWriteCmd(&msg, BL_CALC_CHKSUM);

    if(LCR_SendMsg(&msg) <= 0)
        return -1;

    return 0;

}

int LCR_GetFlashChecksum(unsigned int*checksum)
/**
 * This function works only in prorgamming mode.
 * This function is to be used to retrieve the flash checksum from the controller.
 * LCR_CalculateFlashChecksum() and LCR_WaitForFlashReady() must be called before using this API.
 *
 * @param checksum - O - variable in which the flash checksum is to be returned
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;
#if 0
    LCR_PrepWriteCmd(&msg, BL_CALC_CHKSUM);

    if(LCR_SendMsg(&msg) <= 0)
        return -1;

    LCR_WaitForFlashReady();
#endif
    LCR_PrepReadCmd(BL_GET_CHKSUM);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *checksum = msg.text.data[6];
        *checksum |= (unsigned int)msg.text.data[7] << 8;
        *checksum |= (unsigned int)msg.text.data[8] << 16;
        *checksum |= (unsigned int)msg.text.data[9] << 24;
        return 0;
    }
    return -1;
}

int LCR_GetStatus(unsigned char *pHWStatus, unsigned char *pSysStatus, unsigned char *pMainStatus)
/**
 * This function is to be used to check the various status indicators from the controller.
 * Refer to DLPC350 Programmer's guide section 2.1 "DLPC350 Status Commands" for detailed description of each byte.
 *
 * @param pHWStatus - O - provides status information on the DLPC350's sequencer, DMD controller and initialization.
 * @param pSysStatus - O - provides DLPC350 status on internal memory tests..
 * @param pMainStatus - O - provides DMD park status and DLPC350 sequencer, frame buffer, and gamma correction status.
 *
 * @return 0 PASS <BR>
 *         -1 FAIL <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(STATUS_HW);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pHWStatus = msg.text.data[0];
    }
    else
        return -1;

    LCR_PrepReadCmd(STATUS_SYS);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pSysStatus = msg.text.data[0];
    }
    else
        return -1;

    LCR_PrepReadCmd(STATUS_MAIN);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *pMainStatus = msg.text.data[0];
    }
    else
        return -1;

    return 0;
}

int LCR_SoftwareReset(void)
/**
 * Use this API to reset the controller
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepWriteCmd(&msg, SW_RESET);

    return LCR_SendMsg(&msg);
}

int LCR_SetMode(bool SLmode)
/**
 * The Display Mode Selection Command enables the DLPC350 internal image processing functions for
 * video mode or bypasses them for pattern display mode. This command selects between video or pattern
 * display mode of operation.
 *
 * @param   SLmode  - I - TRUE = Pattern Display mode. Assumes a 1-bit through 8-bit image with a pixel
 *                              resolution of 912 x 1140 and bypasses all the image processing functions of DLPC350
 *                          FALSE = Video Display mode. Assumes streaming video image from the 30-bit
 *                              RGB or FPD-link interface with a pixel resolution of up to 1280 x 800 up to 120 Hz.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = SLmode;
    LCR_PrepWriteCmd(&msg, DISP_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetMode(bool *pMode)
/**
 * The Display Mode Selection Command enables the DLPC350 internal image processing functions for
 * video mode or bypasses them for pattern display mode. This command selects between video or pattern
 * display mode of operation.
 *
 * @param   SLmode  - O - TRUE = Pattern Display mode. Assumes a 1-bit through 8-bit image with a pixel
 *                              resolution of 912 x 1140 and bypasses all the image processing functions of DLPC350
 *                        FALSE = Video Display mode. Assumes streaming video image from the 30-bit
 *                              RGB or FPD-link interface with a pixel resolution of up to 1280 x 800 up to 120 Hz.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(DISP_MODE);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pMode = (msg.text.data[0] != 0);
        return 0;
    }
    return -1;
}

int LCR_SetPowerMode(bool Standby)
/**
 * (I2C: 0x07)
 * (USB: CMD2: 0x02, CMD3: 0x00)
 * The Power Control places the DLPC350 in a low-power state and powers down the DMD interface.
 * Standby mode should only be enabled after all data for the last frame to be displayed has been
 * transferred to the DLPC350. Standby mode must be disabled prior to sending any new data.
 *
 * @param   Standby  - I - TRUE = Standby mode. Places DLPC350 in low power state and powers down the DMD interface
 *                         FALSE = Normal operation. The selected external source will be displayed
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = Standby;
    LCR_PrepWriteCmd(&msg, POWER_CONTROL);

    return LCR_SendMsg(&msg);
}

int LCR_SetRedLEDStrobeDelay(unsigned char rising, unsigned char falling)
/**
 * (I2C: 0x6C)
 * (USB: CMD2: 0x1A, CMD3: 0x1F)
 * The Red LED Enable Delay Control command sets the rising and falling edge delay of the Red LED enable signal.
 *
 * @param   rising  - I - Red LED enable rising edge delay control. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 * @param   falling  - I - Red LED enable falling edge delay control. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = rising;
    msg.text.data[3] = falling;

    LCR_PrepWriteCmd(&msg, RED_STROBE_DLY);

    return LCR_SendMsg(&msg);
}

int LCR_SetGreenLEDStrobeDelay(unsigned char rising, unsigned char falling)
/**
 * (I2C: 0x6D)
 * (USB: CMD2: 0x1A, CMD3: 0x20)
 * The Green LED Enable Delay Control command sets the rising and falling edge delay of the Green LED enable signal.
 *
 * @param   rising  - I - Green LED enable rising edge delay control. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 * @param   falling  - I - Green LED enable falling edge delay control. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = rising;
    msg.text.data[3] = falling;

    LCR_PrepWriteCmd(&msg, GRN_STROBE_DLY);

    return LCR_SendMsg(&msg);
}

int LCR_SetBlueLEDStrobeDelay(unsigned char rising, unsigned char falling)
/**
 * (I2C: 0x6E)
 * (USB: CMD2: 0x1A, CMD3: 0x21)
 * The Blue LED Enable Delay Control command sets the rising and falling edge delay of the Blue LED enable signal.
 *
 * @param   rising  - I - Blue LED enable rising edge delay control. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 * @param   falling  - I - Blue LED enable falling edge delay control. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = rising;
    msg.text.data[3] = falling;

    LCR_PrepWriteCmd(&msg, BLU_STROBE_DLY);

    return LCR_SendMsg(&msg);
}

int LCR_GetRedLEDStrobeDelay(unsigned char *pRising, unsigned char *pFalling)
/**
 * (I2C: 0x6C)
 * (USB: CMD2: 0x1A, CMD3: 0x1F)
 * This command reads back the rising and falling edge delay of the Red LED enable signal.
 *
 * @param   pRising  - O - Red LED enable rising edge delay value. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 * @param   pFalling  - O - Red LED enable falling edge delay value. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(RED_STROBE_DLY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRising = msg.text.data[0];
        *pFalling = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_GetGreenLEDStrobeDelay(unsigned char *pRising, unsigned char *pFalling)
/**
 * (I2C: 0x6D)
 * (USB: CMD2: 0x1A, CMD3: 0x20)
 * This command reads back the rising and falling edge delay of the Green LED enable signal.
 *
 * @param   pRising  - O - Green LED enable rising edge delay value. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 * @param   pFalling  - O - Green LED enable falling edge delay value. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(GRN_STROBE_DLY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRising = msg.text.data[0];
        *pFalling = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_GetBlueLEDStrobeDelay(unsigned char *pRising, unsigned char *pFalling)
/**
 * (I2C: 0x6E)
 * (USB: CMD2: 0x1A, CMD3: 0x21)
 * This command reads back the rising and falling edge delay of the Blue LED enable signal.
 *
 * @param   pRising  - O - Blue LED enable rising edge delay value. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 * @param   pFalling  - O - Blue LED enable falling edge delay value. Each bit adds 107.2 ns.
 *                        0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xFE = +7.1828 μs, 0xFF = +7.29 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(BLU_STROBE_DLY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRising = msg.text.data[0];
        *pFalling = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_SetInputSource(unsigned int source, unsigned int portWidth)
/**
 * (I2C: 0x00)
 * (USB: CMD2: 0x1A, CMD3: 0x00)
 * The Input Source Selection command selects the input source to be displayed by the DLPC350: 30-bit
 * Parallel Port, Internal Test Pattern, Flash memory, or FPD-link interface.
 *
 * @param   source  - I - Select the input source and interface mode:
 *                        0 = Parallel interface with 8-bit, 16-bit, 20-bit, 24-bit, or 30-bit RGB or YCrCb data formats
 *                        1 = Internal test pattern; Use LCR_SetTPGSelect() API to select pattern
 *                        2 = Flash. Images are 24-bit single-frame, still images stored in flash that are uploaded on command.
 *                        3 = FPD-link interface
 * @param   portWidth  - I - Parallel Interface bit depth
 *                           0 = 30-bits
 *                           1 = 24-bits
 *                           2 = 20-bits
 *                           3 = 16-bits
 *                           4 = 10-bits
 *                           5 = 8-bits
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = source;
    msg.text.data[2] |= portWidth << 3;
    LCR_PrepWriteCmd(&msg, SOURCE_SEL);

    return LCR_SendMsg(&msg);
}

int LCR_GetInputSource(unsigned int *pSource, unsigned int *pPortWidth)
/**
 * (I2C: 0x00)
 * (USB: CMD2: 0x1A, CMD3: 0x00)
 * Thisn command reads back the input source to be displayed by the DLPC350
 *
 * @param   pSource  - O - Input source and interface mode:
 *                        0 = Parallel interface with 8-bit, 16-bit, 20-bit, 24-bit, or 30-bit RGB or YCrCb data formats
 *                        1 = Internal test pattern; Use LCR_SetTPGSelect() API to select pattern
 *                        2 = Flash. Images are 24-bit single-frame, still images stored in flash that are uploaded on command.
 *                        3 = FPD-link interface
 * @param   pPortWidth  - O - Parallel Interface bit depth
 *                           0 = 30-bits
 *                           1 = 24-bits
 *                           2 = 20-bits
 *                           3 = 16-bits
 *                           4 = 10-bits
 *                           5 = 8-bits
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(SOURCE_SEL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pSource = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        *pPortWidth = msg.text.data[0] >> 3;
        return 0;
    }
    return -1;
}

int LCR_SetPatternDisplayMode(bool external)
/**
 * The Pattern Display Data Input Source command selects the source of the data for pattern display:
 * streaming through the 24-bit RGB/FPD-link interface or stored data in the splash image memory area from
 * external Flash. Before executing this command, stop the current pattern sequence. After executing this
 * command, send the Validation command (I2C: 0x7D or USB: 0x1A1A) once before starting the pattern
 * sequence.
 *
 * @param   external  - I - TRUE = Pattern Display Data is streamed through the 24-bit RGB/FPD-link interface
 *                          FALSE = Pattern Display Data is fetched from splash memory
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    if(external)
        msg.text.data[2] = 0;
    else
        msg.text.data[2] = 3;

    LCR_PrepWriteCmd(&msg, PAT_DISP_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetPatternDisplayMode(bool *external)
/**
 * The Pattern Display Data Input Source command selects the source of the data for pattern display:
 * streaming through the 24-bit RGB/FPD-link interface or stored data in the splash image memory area from
 * external Flash. Before executing this command, stop the current pattern sequence. After executing this
 * command, send the Validation command (I2C: 0x7D or USB: 0x1A1A) once before starting the pattern
 * sequence.
 *
 * @param   external  - O - TRUE = Pattern Display Data is streamed through the 24-bit RGB/FPD-link interface
 *                          FALSE = Pattern Display Data is fetched from splash memory
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PAT_DISP_MODE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        if(msg.text.data[0] == 0)
            *external = true;
        else
            *external = false;
        return 0;
    }
    return -1;
}

int LCR_SetPixelFormat(unsigned int format)
/**
 * (I2C: 0x02)
 * (USB: CMD2: 0x1A, CMD3: 0x02)
 * This API defines the pixel data format input into the DLPC350.Refer to programmer's guide for supported pixel formats
 * for each source type.
 *
 * @param   format  - I - Select the pixel data format:
 *                        0 = RGB 4:4:4 (30-bit)
 *                        1 = YCrCb 4:4:4 (30-bit)
 *                        2 = YCrCb 4:2:2
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = format;
    LCR_PrepWriteCmd(&msg, PIXEL_FORMAT);

    return LCR_SendMsg(&msg);
}

int LCR_GetPixelFormat(unsigned int *pFormat)
/**
 * (I2C: 0x02)
 * (USB: CMD2: 0x1A, CMD3: 0x02)
 * This API returns the defined the pixel data format input into the DLPC350.Refer to programmer's guide for supported pixel formats
 * for each source type.
 *
 * @param   pFormat  - O - Pixel data format:
 *                        0 = RGB 4:4:4 (30-bit)
 *                        1 = YCrCb 4:4:4 (30-bit)
 *                        2 = YCrCb 4:2:2
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PIXEL_FORMAT);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pFormat = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        return 0;
    }
    return -1;
}

int LCR_SetPortClock(unsigned int clock)
/**
 * (I2C: 0x03)
 * (USB: CMD2: 0x1A, CMD3: 0x03)
 * This API selects the Port 1 clock for the parallel interface. For the FPD-Link, the Port Clock is
 * automatically set to Port 2.
 *
 * @param   clock  - I - Selects the port input clock:
 *                        0 = Port1, clock A
 *                        1 = Port1, clock B
 *                        2 = Port1, clock C
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = clock;
    LCR_PrepWriteCmd(&msg, CLK_SEL);

    return LCR_SendMsg(&msg);
}

int LCR_GetPortClock(unsigned int *pClock)
/**
 * (I2C: 0x03)
 * (USB: CMD2: 0x1A, CMD3: 0x03)
 * This API reads the Port 1 clock for the parallel interface.
 *
 * @param   pClock  - O - Selected port input clock:
 *                        0 = Port1, clock A
 *                        1 = Port1, clock B
 *                        2 = Port1, clock C
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(CLK_SEL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pClock = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        return 0;
    }
    return -1;
}

int LCR_SetDataChannelSwap(unsigned int port, unsigned int swap)
/**
 * (I2C: 0x04)
 * (USB: CMD2: 0x1A, CMD3: 0x37)
 * This API configures the specified input data port and map the data subchannels.
 * The DLPC350 interprets Channel A as Green, Channel B as Red, and Channel C as Blue.
 *
 * @param   port  - I - Selects the port:
 *                        0 = Port1, parallel interface
 *                        1 = Port2, FPD-link interface
 * @param   swap - I - Swap Data Sub-Channel:
 *                     0 - ABC = ABC, No swapping of data sub-channels
 *                     1 - ABC = CAB, Data sub-channels are right shifted and circularly rotated
 *                     2 - ABC = BCA, Data sub-channels are left shifted and circularly rotated
 *                     3 - ABC = ACB, Data sub-channels B and C are swapped
 *                     4 - ABC = BAC, Data sub-channels A and B are swapped
 *                     5 - ABC = CBA, Data sub-channels A and C are swapped
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = port << 7;
    msg.text.data[2] |= swap & (BIT0 | BIT1 | BIT2);
    LCR_PrepWriteCmd(&msg, CHANNEL_SWAP);

    return LCR_SendMsg(&msg);
}

int LCR_GetDataChannelSwap(unsigned int *pPort, unsigned int *pSwap)
/**
 * (I2C: 0x04)
 * (USB: CMD2: 0x1A, CMD3: 0x37)
 * This API reads the data subchannel mapping for the specified input data port and map the data subchannels
 *
 * @param   *pPort  - O - Selected port:
 *                        0 = Port1, parallel interface
 *                        1 = Port2, FPD-link interface
 * @param   *pSwap - O - Swap Data Sub-Channel:
 *                     0 - ABC = ABC, No swapping of data sub-channels
 *                     1 - ABC = CAB, Data sub-channels are right shifted and circularly rotated
 *                     2 - ABC = BCA, Data sub-channels are left shifted and circularly rotated
 *                     3 - ABC = ACB, Data sub-channels B and C are swapped
 *                     4 - ABC = BAC, Data sub-channels A and B are swapped
 *                     5 - ABC = CBA, Data sub-channels A and C are swapped
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(CHANNEL_SWAP);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pSwap = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        if(msg.text.data[0] & BIT7)
            *pPort = 1;
        else
            *pPort = 0;
        return 0;
    }
    return -1;
}

int LCR_SetFPD_Mode_Field(unsigned int PixelMappingMode, bool SwapPolarity, unsigned int FieldSignalSelect)
/**
 * (I2C: 0x05)
 * (USB: CMD2: 0x1A, CMD3: 0x04)
 * The FPD-Link Mode and Field Select command configures the FPD-link pixel map, polarity, and signal select.
 *
 * @param   PixelMappingMode  - I - FPD-link Pixel Mapping Mode: See table 2-21 in programmer's guide for more details
 *                                  0 = Mode 1
 *                                  1 = Mode 2
 *                                  2 = Mode 3
 *                                  3 = Mode 4
 * @param   SwapPolarity - I - Polarity select
 *                             true = swap polarity
 *                             false = do not swap polarity
 *
 * @param   FieldSignalSelect -I -  Field Signal Select
 *                              0 - Map FPD-Link output from CONT1 onto Field Signal for FPD-link interface port
 *                              1 - Map FPD-Link output from CONT2 onto Field Signal for FPD-link interface port
 *                              2 - Force 0 onto Field Signal for FPD-link interface port
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = PixelMappingMode << 6;
    msg.text.data[2] |= FieldSignalSelect & (BIT0 | BIT1 | BIT2);
    if(SwapPolarity)
        msg.text.data[2] |= BIT3;
    LCR_PrepWriteCmd(&msg, FPD_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetFPD_Mode_Field(unsigned int *pPixelMappingMode, bool *pSwapPolarity, unsigned int *pFieldSignalSelect)
/**
 * (I2C: 0x05)
 * (USB: CMD2: 0x1A, CMD3: 0x04)
 * This command reads back the configuration of FPD-link pixel map, polarity, and signal select.
 *
 * @param   PixelMappingMode  - O - FPD-link Pixel Mapping Mode: See table 2-21 in programmer's guide for more details
 *                                  0 = Mode 1
 *                                  1 = Mode 2
 *                                  2 = Mode 3
 *                                  3 = Mode 4
 * @param   SwapPolarity - O - Polarity select
 *                             true = swap polarity
 *                             false = do not swap polarity
 *
 * @param   FieldSignalSelect - O -  Field Signal Select
 *                              0 - Map FPD-Link output from CONT1 onto Field Signal for FPD-link interface port
 *                              1 - Map FPD-Link output from CONT2 onto Field Signal for FPD-link interface port
 *                              2 - Force 0 onto Field Signal for FPD-link interface port
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(FPD_MODE);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pFieldSignalSelect = msg.text.data[0] & (BIT0 | BIT1 | BIT2);
        if(msg.text.data[0] & BIT3)
            *pSwapPolarity = 1;
        else
            *pSwapPolarity = 0;
        *pPixelMappingMode = msg.text.data[0] >> 6;
        return 0;
    }
    return -1;
}

int LCR_SetTPGSelect(unsigned int pattern)
/**
 * (I2C: 0x0A)
 * (USB: CMD2: 0x12, CMD3: 0x03)
 * When the internal test pattern is the selected input, the Internal Test Patterns Select defines the test
 * pattern displayed on the screen. These test patterns are internally generated and injected into the
 * beginning of the DLPC350 image processing path. Therefore, all image processing is performed on the
 * test images. All command registers should be set up as if the test images are input from an RGB 8:8:8
 * external source.
 *
 * @param   pattern  - I - Selects the internal test pattern:
 *                         0x0 = Solid Field
 *                         0x1 = Horizontal Ramp
 *                         0x2 = Vertical Ramp
 *                         0x3 = Horizontal Lines
 *                         0x4 = Diagonal Lines
 *                         0x5 = Vertical Lines
 *                         0x6 = Grid
 *                         0x7 = Checkerboard
 *                         0x8 = RGB Ramp
 *                         0x9 = Color Bars
 *                         0xA = Step Bars
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = pattern;
    LCR_PrepWriteCmd(&msg, TPG_SEL);

    return LCR_SendMsg(&msg);
}

int LCR_GetTPGSelect(unsigned int *pPattern)
/**
 * (I2C: 0x0A)
 * (USB: CMD2: 0x12, CMD3: 0x03)
 * This command reads back the selected internal test pattern.
 *
 * @param   pattern  - O - Selected internal test pattern:
 *                         0x0 = Solid Field
 *                         0x1 = Horizontal Ramp
 *                         0x2 = Vertical Ramp
 *                         0x3 = Horizontal Lines
 *                         0x4 = Diagonal Lines
 *                         0x5 = Vertical Lines
 *                         0x6 = Grid
 *                         0x7 = Checkerboard
 *                         0x8 = RGB Ramp
 *                         0x9 = Color Bars
 *                         0xA = Step Bars
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(TPG_SEL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pPattern = msg.text.data[0] & (BIT0 | BIT1 | BIT2 | BIT3);
        return 0;
    }
    return -1;
}

int LCR_LoadSplash(unsigned int index)
/**
 * (I2C: 0x7F)
 * (USB: CMD2: 0x1A, CMD3: 0x39)
 * This command loads an image from flash memory and then performs a buffer swap to display the loaded
 * image on the DMD.
 *
 * @param   index  - I - Image Index. Loads the image at this index from flash.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = index;
    LCR_PrepWriteCmd(&msg, SPLASH_LOAD);

    return LCR_SendMsg(&msg);
}

int LCR_GetSplashIndex(unsigned int *pIndex)
/**
 * (I2C: 0x7F)
 * (USB: CMD2: 0x1A, CMD3: 0x39)
 * This command loads reads back the index that was loaded most recently via LCR_LoadSplash() API.
 *
 * @param   *pIndex  - O - Image Index. Image at this index is loaded from flash.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(SPLASH_LOAD);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pIndex = msg.text.data[0];
        return 0;
    }
    return -1;
}

int LCR_SetDisplay(rectangle croppedArea, rectangle displayArea)
/**
 * (I2C: 0x7E)
 * (USB: CMD2: 0x10, CMD3: 0x00)
 * The Input Display Resolution command defines the active input resolution and active output (displayed)
 * resolution. The maximum supported input and output resolutions for the DLP4500 0.45 WXGA DMD is
 * 1280 pixels (columns) by 800 lines (rows). This command provides the option to define a subset of active
 * input frame data using pixel (column) and line (row) counts relative to the source-data enable signal
 * (DATEN). In other words, this feature allows the source image to be cropped as the first step in the
 * processing chain.
 *
 * @param croppedArea - I - The rectagle structure contains the following
 *          parameters to describe the area to be cropped:
 *              - FirstPixel <BR>
 *              - FirstLine <BR>
 *              - PixelsPerLine <BR>
 *              - LinesPerFrame <BR>
 * @param displayArea - I - The rectagle structure contains the following
 *          parameters to describe the display area:
 *              - FirstPixel <BR>
 *              - FirstLine <BR>
 *              - PixelsPerLine <BR>
 *              - LinesPerFrame <BR>
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = croppedArea.firstPixel & 0xFF;
    msg.text.data[3] = croppedArea.firstPixel >> 8;
    msg.text.data[4] = croppedArea.firstLine & 0xFF;
    msg.text.data[5] = croppedArea.firstLine >> 8;
    msg.text.data[6] = croppedArea.pixelsPerLine & 0xFF;
    msg.text.data[7] = croppedArea.pixelsPerLine >> 8;
    msg.text.data[8] = croppedArea.linesPerFrame & 0xFF;
    msg.text.data[9] = croppedArea.linesPerFrame >> 8;
    msg.text.data[10] = displayArea.firstPixel & 0xFF;
    msg.text.data[11] = displayArea.firstPixel >> 8;
    msg.text.data[12] = displayArea.firstLine & 0xFF;
    msg.text.data[13] = displayArea.firstLine >> 8;
    msg.text.data[14] = displayArea.pixelsPerLine & 0xFF;
    msg.text.data[15] = displayArea.pixelsPerLine >> 8;
    msg.text.data[16] = displayArea.linesPerFrame & 0xFF;
    msg.text.data[17] = displayArea.linesPerFrame >> 8;

    LCR_PrepWriteCmd(&msg, DISP_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetDisplay(rectangle *pCroppedArea, rectangle *pDisplayArea)
/**
 * (I2C: 0x7E)
 * (USB: CMD2: 0x10, CMD3: 0x00)
 * This command reads back the active input resolution and active output (displayed) resolution.
 *
 * @param *pCroppedArea - O - The rectagle structure contains the following
 *          parameters to describe the area to be cropped:
 *              - FirstPixel <BR>
 *              - FirstLine <BR>
 *              - PixelsPerLine <BR>
 *              - LinesPerFrame <BR>
 * @param *pDisplayArea - O - The rectagle structure contains the following
 *          parameters to describe the display area:
 *              - FirstPixel <BR>
 *              - FirstLine <BR>
 *              - PixelsPerLine <BR>
 *              - LinesPerFrame <BR>
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(DISP_CONFIG);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        pCroppedArea->firstPixel = msg.text.data[0] | msg.text.data[1] << 8;
        pCroppedArea->firstLine = msg.text.data[2] | msg.text.data[3] << 8;
        pCroppedArea->pixelsPerLine = msg.text.data[4] | msg.text.data[5] << 8;
        pCroppedArea->linesPerFrame = msg.text.data[6] | msg.text.data[7] << 8;
        pDisplayArea->firstPixel = msg.text.data[8] | msg.text.data[9] << 8;
        pDisplayArea->firstLine = msg.text.data[10] | msg.text.data[11] << 8;
        pDisplayArea->pixelsPerLine = msg.text.data[12] | msg.text.data[13] << 8;
        pDisplayArea->linesPerFrame = msg.text.data[14] | msg.text.data[15] << 8;

        return 0;
    }
    return -1;
}

int LCR_SetTPGColor(unsigned short redFG, unsigned short greenFG, unsigned short blueFG, unsigned short redBG, unsigned short greenBG, unsigned short blueBG)
/**
 * (I2C: 0x1A)
 * (USB: CMD2: 0x12, CMD3: 0x04)
 * When the internal test pattern is the selected input, the Internal Test Patterns Color Control defines the
 * colors of the test pattern displayed on the screen. The foreground color setting affects all test patterns. The background color
 * setting affects those test patterns that have a foreground and background component, such as, Horizontal
 * Lines, Diagonal Lines, Vertical Lines, Grid, and Checkerboard.
 *
 * @param   redFG  - I - Red Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Red Foreground color intensity
 *                       0x3FF = Full Red Foreground color intensity
 * @param   greenFG  - I - Green Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Green Foreground color intensity
 *                       0x3FF = Full Green Foreground color intensity
 * @param   blueFG  - I - Blue Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Blue Foreground color intensity
 *                       0x3FF = Full Blue Foreground color intensity
 * @param   redBG  - I - Red Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Red Foreground color intensity
 *                       0x3FF = Full Red Foreground color intensity
 * @param   greenBG  - I - Green Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Green Foreground color intensity
 *                       0x3FF = Full Red Foreground color intensity
 * @param   blueBG  - I - Red Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Blue Foreground color intensity
 *                       0x3FF = Full Blue Foreground color intensity
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = (char)redFG;
    msg.text.data[3] = (char)(redFG >> 8);
    msg.text.data[4] = (char)greenFG;
    msg.text.data[5] = (char)(greenFG >> 8);
    msg.text.data[6] = (char)blueFG;
    msg.text.data[7] = (char)(blueFG >> 8);
    msg.text.data[8] = (char)redBG;
    msg.text.data[9] = (char)(redBG >> 8);
    msg.text.data[10] = (char)greenBG;
    msg.text.data[11] = (char)(greenBG >> 8);
    msg.text.data[12] = (char)blueBG;
    msg.text.data[13] = (char)(blueBG >> 8);

    LCR_PrepWriteCmd(&msg, TPG_COLOR);

    return LCR_SendMsg(&msg);
}

int LCR_GetTPGColor(unsigned short *pRedFG, unsigned short *pGreenFG, unsigned short *pBlueFG, unsigned short *pRedBG, unsigned short *pGreenBG, unsigned short *pBlueBG)
/**
 * (I2C: 0x1A)
 * (USB: CMD2: 0x12, CMD3: 0x04)
 * When the internal test pattern is the selected input, the Internal Test Patterns Color Control defines the
 * colors of the test pattern displayed on the screen. The foreground color setting affects all test patterns. The background color
 * setting affects those test patterns that have a foreground and background component, such as, Horizontal
 * Lines, Diagonal Lines, Vertical Lines, Grid, and Checkerboard.
 *
 * @param   *pRedFG  - O - Red Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Red Foreground color intensity
 *                       0x3FF = Full Red Foreground color intensity
 * @param   *pGreenFG  - O - Green Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Green Foreground color intensity
 *                       0x3FF = Full Green Foreground color intensity
 * @param   *pBlueFG  - O - Blue Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Blue Foreground color intensity
 *                       0x3FF = Full Blue Foreground color intensity
 * @param   *pRedBG  - O - Red Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Red Foreground color intensity
 *                       0x3FF = Full Red Foreground color intensity
 * @param   *pGreenBG  - O - Green Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Green Foreground color intensity
 *                       0x3FF = Full Red Foreground color intensity
 * @param   *pBlueBG  - O - Red Foreground Color intensity in a scale from 0 to 1023
 *                       0x0 = No Blue Foreground color intensity
 *                       0x3FF = Full Blue Foreground color intensity
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(TPG_COLOR);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pRedFG = msg.text.data[0] | msg.text.data[1] << 8;
        *pGreenFG = msg.text.data[2] | msg.text.data[3] << 8;
        *pBlueFG = msg.text.data[4] | msg.text.data[5] << 8;
        *pRedBG = msg.text.data[6] | msg.text.data[7] << 8;
        *pGreenBG = msg.text.data[8] | msg.text.data[9] << 8;
        *pBlueBG = msg.text.data[10] | msg.text.data[11] << 8;

        return 0;
    }
    return -1;
}

int LCR_ClearPatLut(void)
/**
 * This API does not send any commands to the controller.It clears the locally (in the GUI program) stored pattern LUT.
 * See table 2-65 in programmer's guide for detailed desciprtion of pattern LUT entries.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    PatLutIndex = 0;
    return 0;
}

int LCR_AddToPatLut(int TrigType, int PatNum,int BitDepth,int LEDSelect,bool InvertPat, bool InsertBlack,bool BufSwap, bool trigOutPrev)
/**
 * This API does not send any commands to the controller.
 * It makes an entry (appends) in the locally stored (in the GUI program) pattern LUT as per the input arguments passed to this function.
 * See table 2-65 in programmer's guide for detailed desciprtion of pattern LUT entries.
 *
 * @param   TrigType  - I - Select the trigger type for the pattern
 *                          0 = Internal trigger
 *                          1 = External positive
 *                          2 = External negative
 *                          3 = No Input Trigger (Continue from previous; Pattern still has full exposure time)
 *                       0x3FF = Full Red Foreground color intensity
 * @param   PatNum  - I - Pattern number (0 based index). For pattern number 0x3F, there is no
 *                          pattern display. The maximum number supported is 24 for 1 bit-depth
 *                          patterns. Setting the pattern number to be 25, with a bit-depth of 1 will insert
 *                          a white-fill pattern. Inverting this pattern will insert a black-fill pattern. These w
 *                          patterns will have the same exposure time as defined in the Pattern Display
 *                          Exposure and Frame Period command. Table 2-66 in the programmer's guide illustrates which bit
 *                          planes are illuminated by each pattern number.
 * @param   BitDepth  - I - Select desired bit-depth
 *                          0 = Reserved
 *                          1 = 1-bit
 *                          2 = 2-bit
 *                          3 = 3-bit
 *                          4 = 4-bit
 *                          5 = 5-bit
 *                          6 = 6-bit
 *                          7 = 7-bit
 *                          8 = 8-bit
 * @param   LEDSelect  - I -  Choose the LEDs that are on: b0 = Red, b1 = Green, b2 = Blue
 *                          0 = No LED (Pass Through)
 *                          1 = Red
 *                          2 = Green
 *                          3 = Yellow (Green + Red)
 *                          4 = Blue
 *                          5 = Magenta (Blue + Red)
 *                          6 = Cyan (Blue + Green)
 *                          7 = White (Red + Blue + Green)
 * @param   InvertPat  - I - true = Invert pattern
 *                           false = do not invert pattern
 * @param   InsertBlack  - I - true = Insert black-fill pattern after current pattern. This setting requires 230 μs
 *                                      of time before the start of the next pattern
 *                           false = do not insert any post pattern
 * @param   BufSwap  - I - true = perform a buffer swap
 *                           false = do not perform a buffer swap
 * @param   trigOutPrev  - I - true = Trigger Out 1 will continue to be high. There will be no falling edge
 *                                       between the end of the previous pattern and the start of the current pattern. w
 *                                       Exposure time is shared between all patterns defined under a common
 *                                       trigger out). This setting cannot be combined with the black-fill pattern
 *                           false = Trigger Out 1 has a rising edge at the start of a pattern, and a falling
 *                                       edge at the end of the pattern
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    unsigned int lutWord = 0;

    lutWord = TrigType & 3;
    if(PatNum > 24)
        return -1;

    lutWord |= ((PatNum & 0x3F) << 2);
    if( (BitDepth > 8) || (BitDepth <= 0))
        return -1;
    lutWord |= ((BitDepth & 0xF) << 8);
    if(LEDSelect > 7)
        return -1;
    lutWord |= ((LEDSelect & 0x7) << 12);
    if(InvertPat)
        lutWord |= BIT16;
    if(InsertBlack)
        lutWord |= BIT17;
    if(BufSwap)
        lutWord |= BIT18;
    if(trigOutPrev)
        lutWord |= BIT19;

    PatLut[PatLutIndex++] = lutWord;
    return 0;
}

int LCR_GetPatLutItem(int index, int *pTrigType, int *pPatNum,int *pBitDepth,int *pLEDSelect,bool *pInvertPat, bool *pInsertBlack,bool *pBufSwap, bool *pTrigOutPrev)
/**
 * This API does not send any commands to the controller.
 * It reads back an entry at the specified index from the locally stored (in the GUI program) pattern LUT and populates the input arguments passed to this function.
 * See table 2-65 in programmer's guide for detailed desciprtion of pattern LUT entries.
 *
 * @param   index  - I - Entry at this index from pattern LUT to be read back.
 * @param   *pTrigType  - O - Select the trigger type for the pattern
 *                          0 = Internal trigger
 *                          1 = External positive
 *                          2 = External negative
 *                          3 = No Input Trigger (Continue from previous; Pattern still has full exposure time)
 *                       0x3FF = Full Red Foreground color intensity
 * @param   *pPatNum  - O - Pattern number (0 based index). For pattern number 0x3F, there is no
 *                          pattern display. The maximum number supported is 24 for 1 bit-depth
 *                          patterns. Setting the pattern number to be 25, with a bit-depth of 1 will insert
 *                          a white-fill pattern. Inverting this pattern will insert a black-fill pattern. These w
 *                          patterns will have the same exposure time as defined in the Pattern Display
 *                          Exposure and Frame Period command. Table 2-66 in the programmer's guide illustrates which bit
 *                          planes are illuminated by each pattern number.
 * @param   *pBitDepth  - O - Select desired bit-depth
 *                          0 = Reserved
 *                          1 = 1-bit
 *                          2 = 2-bit
 *                          3 = 3-bit
 *                          4 = 4-bit
 *                          5 = 5-bit
 *                          6 = 6-bit
 *                          7 = 7-bit
 *                          8 = 8-bit
 * @param   *pLEDSelect  - O -  Choose the LEDs that are on: b0 = Red, b1 = Green, b2 = Blue
 *                          0 = No LED (Pass Through)
 *                          1 = Red
 *                          2 = Green
 *                          3 = Yellow (Green + Red)
 *                          4 = Blue
 *                          5 = Magenta (Blue + Red)
 *                          6 = Cyan (Blue + Green)
 *                          7 = White (Red + Blue + Green)
 * @param   *pInvertPat  - O - true = Invert pattern
 *                           false = do not invert pattern
 * @param   *pInsertBlack  - O - true = Insert black-fill pattern after current pattern. This setting requires 230 μs
 *                                      of time before the start of the next pattern
 *                           false = do not insert any post pattern
 * @param   *pBufSwap  - O - true = perform a buffer swap
 *                           false = do not perform a buffer swap
 * @param   *pTrigOutPrev  - O - true = Trigger Out 1 will continue to be high. There will be no falling edge
 *                                       between the end of the previous pattern and the start of the current pattern. w
 *                                       Exposure time is shared between all patterns defined under a common
 *                                       trigger out). This setting cannot be combined with the black-fill pattern
 *                           false = Trigger Out 1 has a rising edge at the start of a pattern, and a falling
 *                                       edge at the end of the pattern
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    unsigned int lutWord;

    lutWord = PatLut[index];

    *pTrigType = lutWord & 3;
    *pPatNum = (lutWord >> 2) & 0x3F;
    *pBitDepth = (lutWord >> 8) & 0xF;
    *pLEDSelect = (lutWord >> 12) & 7;
    *pInvertPat = ((lutWord & BIT16) == BIT16);
    *pInsertBlack = ((lutWord & BIT17) == BIT17);
    *pBufSwap = ((lutWord & BIT18) == BIT18);
    *pTrigOutPrev = ((lutWord & BIT19) == BIT19);

    return 0;
}

int LCR_OpenMailbox(int MboxNum)
/**
 * (I2C: 0x77)
 * (USB: CMD2: 0x1A, CMD3: 0x33)
 * This API opens the specified Mailbox within the DLPC350 controller. This API must be called
 * before sending data to the mailbox/LUT using LCR_SendPatLut() or LCR_SendSplashLut() APIs.
 *
 * @param MboxNum - I - 1 = Open the mailbox for image index configuration
 *                      2 = Open the mailbox for pattern definition.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = MboxNum;
    LCR_PrepWriteCmd(&msg, MBOX_CONTROL);

    return LCR_SendMsg(&msg);
}

int LCR_CloseMailbox(void)
/**
 * (I2C: 0x77)
 * (USB: CMD2: 0x1A, CMD3: 0x33)
 * This API is internally used by other APIs within this file. There is no need for user application to
 * call this API separately.
 * This API closes all the Mailboxes within the DLPC350 controller.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = 0;
    LCR_PrepWriteCmd(&msg, MBOX_CONTROL);

    return LCR_SendMsg(&msg);
}

int LCR_MailboxSetAddr(int Addr)
/**
 * (I2C: 0x76)
 * (USB: CMD2: 0x1A, CMD3: 0x32)
 * This API defines the offset location within the DLPC350 mailboxes to write data into or to read data from
 *
 * @param Addr - I - 0-127 - Defines the offset within the selected (opened) LUT to write/read data to/from.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    if(Addr > 127)
        return -1;

    msg.text.data[2] = Addr;
    LCR_PrepWriteCmd(&msg, MBOX_ADDRESS);

    return LCR_SendMsg(&msg);
}

int LCR_SendPatLut(void)
/**
 * (I2C: 0x78)
 * (USB: CMD2: 0x1A, CMD3: 0x34)
 * This API sends the pattern LUT created by calling LCR_AddToPatLut() API to the DLPC350 controller.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;
    int bytesToSend=PatLutIndex*3;
    unsigned int i;

    if(LCR_OpenMailbox(2) < 0)
        return -1;
    LCR_MailboxSetAddr(0);

    CmdList[MBOX_DATA].len = bytesToSend;
    LCR_PrepWriteCmd(&msg, MBOX_DATA);

    for(i=0; i<PatLutIndex; i++)
    {
        msg.text.data[2+3*i] = PatLut[i];
        msg.text.data[2+3*i+1] = PatLut[i]>>8;
        msg.text.data[2+3*i+2] = PatLut[i]>>16;
    }

    LCR_SendMsg(&msg);
    LCR_CloseMailbox();
    return 0;
}

int LCR_SendSplashLut(unsigned char *lutEntries, unsigned int numEntries)
/**
 * (I2C: 0x78)
 * (USB: CMD2: 0x1A, CMD3: 0x34)
 * This API sends the image LUT to the DLPC350 controller.
 *
 * @param   *lutEntries - I - Pointer to the array in which LUT entries to be sent are stored
 *
 * @param   numEntries - I - number of entries to be sent to the controller
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;
    unsigned int i;

    if(numEntries < 1 || numEntries > 64)
        return -1;

    LCR_OpenMailbox(1);
    LCR_MailboxSetAddr(0);

    // Check for special case of 2 entries
    if( numEntries == 2)
    {
         msg.text.data[2+0] = lutEntries[1];
         msg.text.data[2+1] = lutEntries[0];
    }
    else
    {
        for(i=0; i < numEntries; i++)
        {
            msg.text.data[2+i] = lutEntries[i];
        }
    }

    CmdList[MBOX_DATA].len = numEntries;
    LCR_PrepWriteCmd(&msg, MBOX_DATA);
    LCR_SendMsg(&msg);
    LCR_CloseMailbox();

    return 0;
}

int LCR_GetPatLut(int numEntries)
/**
 * (I2C: 0x78)
 * (USB: CMD2: 0x1A, CMD3: 0x34)
 * This API reads the pattern LUT from the DLPC350 controller and stores it in the local array.
 * The pattern LUT entries could be queried using the API LCR_GetPatLutItem().
 *
 * @param   numEntries - I - Number of entries expected in pattern LUT.
 *
 * @return  number of bytes actually read from the controller LUT.
 *
 */
{
    hidMessageStruct msg;
    unsigned int lutWord = 0;
    int numBytes, i;
    unsigned char *readBuf;

    if(numEntries > 128)
        return -1;

    if(LCR_OpenMailbox(2) < 0)
        return -1;

    if(LCR_MailboxSetAddr(0) < 0)
        return -1;

    numBytes = sizeof(msg.head)+numEntries*3;
    readBuf = (unsigned char *)&msg;
    LCR_PrepReadCmd(MBOX_DATA);


    if(LCR_Read() > 0)
    {
        memcpy(readBuf, InputBuffer, MIN(numBytes,64));
        readBuf+=64;
        numBytes -=64;
    }
    else
    {
        LCR_CloseMailbox();
        return -1;
    }
    /* If packet is greater than 64 bytes, continue to read */
    while(numBytes > 0)
    {
        if(LCR_ContinueRead() < 0)
            return -1;
        memcpy(readBuf, InputBuffer, MIN(numBytes,64));
        readBuf+=64;
        numBytes -=64;
    }

    LCR_ClearPatLut();

    for(i=0; i<numEntries*3; i+=3)
    {
        lutWord = msg.text.data[i] | msg.text.data[i+1] << 8 | msg.text.data[i+2] << 16;
        PatLut[PatLutIndex++] = lutWord;
    }

    if(LCR_CloseMailbox() < 0)
        return -1;

    return (int)msg.head.length;
}

int LCR_GetSplashLut(unsigned char *pLut, int numEntries)
/**
 * (I2C: 0x78)
 * (USB: CMD2: 0x1A, CMD3: 0x34)
 * This API reads the image LUT from the DLPC350 controller.
 *
 * @param   *pLut - I - Pointer to the array in which the read entries should be stored
 *
 * @param   numEntries - I - Number of image LUT entries to be read from the controller
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    int retval;

    hidMessageStruct *pMsg;
    if(LCR_OpenMailbox(1) < 0)
        return -1;

    if(LCR_MailboxSetAddr(0) < 0)
        return -1;

    LCR_PrepReadCmd(MBOX_DATA);

    if((retval = LCR_Read()) > 0)
    {
        memcpy(pLut, InputBuffer+sizeof(pMsg->head), MIN(numEntries,64-sizeof(pMsg->head)));
        pLut+= (64-sizeof(pMsg->head));
        numEntries -= (64-sizeof(pMsg->head));
    }
    else
    {
        LCR_CloseMailbox();
        return retval;
    }

    /* If packet is greater than 64 bytes, continue to read */
    while(numEntries > 0)
    {
        LCR_ContinueRead();
        memcpy(pLut, InputBuffer, MIN(numEntries,64));
        pLut+=64;
        numEntries -= 64;
    }

    if(LCR_CloseMailbox() < 0)
        return -1;

    return 0;
}

int LCR_SetPatternTriggerMode(bool IntExt_or_Vsync)
/**
 * The Pattern Trigger Mode Selection command selects between one of the three pattern Trigger Modes.
 * Before executing this command, stop the current pattern sequence. After executing this command, send
 * the Validation command (I2C: 0x7D or USB: 0x1A1A) once before starting the pattern sequence.
 *
 * @param   IntExt_or_Vsync  - I - 1 = Pattern Trigger Mode 1: Internally or Externally (through TRIG_IN1 and TRIG_IN2) generated trigger
 *                                    0 = Pattern Trigger Mode 0: VSYNC serves to trigger the pattern display sequence.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = IntExt_or_Vsync;
    LCR_PrepWriteCmd(&msg, PAT_TRIG_MODE);

    return LCR_SendMsg(&msg);
}

int LCR_GetPatternTriggerMode(bool *IntExt_or_Vsync)
/**
 * The Pattern Trigger Mode Selection command selects between one of the three pattern Trigger Modes.
 *
 * @param   IntExt_or_Vsync  -0 - 1 = Pattern Trigger Mode 1: Internally or Externally (through TRIG_IN1 and TRIG_IN2) generated trigger
 *                                    0 = Pattern Trigger Mode 0: VSYNC serves to trigger the pattern display sequence.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{    hidMessageStruct msg;

     LCR_PrepReadCmd(PAT_TRIG_MODE);

     if(LCR_Read() > 0)
     {
         memcpy(&msg, InputBuffer, 65);
         *IntExt_or_Vsync = (msg.text.data[0] != 0);
         return 0;
     }
     return -1;
}


int LCR_PatternDisplay(int Action)
/**
 * (I2C: 0x65)
 * (USB: CMD2: 0x1A, CMD3: 0x24)
 * This API starts or stops the programmed patterns sequence.
 *
 * @param   Action - I - Pattern Display Start/Stop Pattern Sequence
 *                          0 = Stop Pattern Display Sequence. The next "Start" command will
 *                              restart the pattern sequence from the beginning.
 *                          1 = Pause Pattern Display Sequence. The next "Start" command will
 *                              start the pattern sequence by re-displaying the current pattern in the sequence.
 *                          2 = Start Pattern Display Sequence
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = Action;
    LCR_PrepWriteCmd(&msg, PAT_START_STOP);

    return LCR_SendMsg(&msg);
}

int LCR_SetPatternConfig(unsigned int numLutEntries, bool repeat, unsigned int numPatsForTrigOut2, unsigned int numSplash)
/**
 * (I2C: 0x75)
 * (USB: CMD2: 0x1A, CMD3: 0x31)
 * This API controls the execution of patterns stored in the lookup table.
 * Before using this API, stop the current pattern sequence using LCR_PatternDisplay() API
 * After calling this API, send the Validation command using the API LCR_ValidatePatLutData() before starting the pattern sequence
 *
 * @param   numLutEntries - I - Number of LUT entries
 * @param   repeat - I - 0 = execute the pattern sequence once; 1 = repeat the pattern sequnce.
 * @param   numPatsForTrigOut2 - I - Number of patterns to display(range 1 through 256).
 *                                   If in repeat mode, then this value dictates how often TRIG_OUT_2 is generated.
 * @param   numSplash - I - Number of Image Index LUT Entries(range 1 through 64).
 *                          This Field is irrelevant for Pattern Display Data Input Source set to a value other than internal.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = numLutEntries-1; /* -1 because the firmware command takes 0-based indices (0 means 1) */
    msg.text.data[3] = repeat;
    msg.text.data[4] = numPatsForTrigOut2 - 1;  /* -1 because the firmware command takes 0-based indices (0 means 1) */
    msg.text.data[5] = numSplash - 1;   /* -1 because the firmware command takes 0-based indices (0 means 1) */
    LCR_PrepWriteCmd(&msg, PAT_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetPatternConfig(unsigned int *pNumLutEntries, bool *pRepeat, unsigned int *pNumPatsForTrigOut2, unsigned int *pNumSplash)
/**
 * (I2C: 0x75)
 * (USB: CMD2: 0x1A, CMD3: 0x31)
 * This API controls the execution of patterns stored in the lookup table.
 * Before using this API, stop the current pattern sequence using LCR_PatternDisplay() API
 * After calling this API, send the Validation command using the API LCR_ValidatePatLutData() before starting the pattern sequence
 *
 * @param   *pNumLutEntries - O - Number of LUT entries
 * @param   *pRepeat - O - 0 = execute the pattern sequence once; 1 = repeat the pattern sequnce.
 * @param   *pNumPatsForTrigOut2 - O - Number of patterns to display(range 1 through 256).
 *                                   If in repeat mode, then this value dictates how often TRIG_OUT_2 is generated.
 * @param   *pNumSplash - O - Number of Image Index LUT Entries(range 1 through 64).
 *                          This Field is irrelevant for Pattern Display Data Input Source set to a value other than internal.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PAT_CONFIG);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pNumLutEntries = msg.text.data[0] + 1; /* +1 because the firmware gives 0-based indices (0 means 1) */
        *pRepeat = (msg.text.data[1] != 0);
        *pNumPatsForTrigOut2 = msg.text.data[2]+1;    /* +1 because the firmware gives 0-based indices (0 means 1) */
        *pNumSplash = msg.text.data[3]+1;     /* +1 because the firmware gives 0-based indices (0 means 1) */
        return 0;
    }
    return -1;
}

int LCR_SetExposure_FramePeriod(unsigned int exposurePeriod, unsigned int framePeriod)
/**
 * (I2C: 0x66)
 * (USB: CMD2: 0x1A, CMD3: 0x29)
 * The Pattern Display Exposure and Frame Period dictates the time a pattern is exposed and the frame
 * period. Either the exposure time must be equivalent to the frame period, or the exposure time must be
 * less than the frame period by 230 microseconds. Before executing this command, stop the current pattern
 * sequence. After executing this command, call LCR_ValidatePatLutData() API before starting the pattern sequence.
 *
 * @param   exposurePeriod - I - Exposure time in microseconds.
 * @param   framePeriod - I - Frame period in microseconds.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
        hidMessageStruct msg;

        msg.text.data[2] = exposurePeriod;
        msg.text.data[3] = exposurePeriod>>8;
        msg.text.data[4] = exposurePeriod>>16;
        msg.text.data[5] = exposurePeriod>>24;
        msg.text.data[6] = framePeriod;
        msg.text.data[7] = framePeriod>>8;
        msg.text.data[8] = framePeriod>>16;
        msg.text.data[9] = framePeriod>>24;
        LCR_PrepWriteCmd(&msg, PAT_EXPO_PRD);

        return LCR_SendMsg(&msg);
}

int LCR_GetExposure_FramePeriod(unsigned int *pExposure, unsigned int *pFramePeriod)
/**
 * (I2C: 0x66)
 * (USB: CMD2: 0x1A, CMD3: 0x29)
 * This API reads back the exposure time and frame period settings from the controller.
 *
 * @param   exposurePeriod - O - Exposure time in microseconds.
 * @param   framePeriod - O - Frame period in microseconds.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PAT_EXPO_PRD);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pExposure = msg.text.data[0] | msg.text.data[1] << 8 | msg.text.data[2] << 16 | msg.text.data[3] << 24;
        *pFramePeriod = msg.text.data[4] | msg.text.data[5] << 8 | msg.text.data[6] << 16 | msg.text.data[7] << 24;
        return 0;
    }
    return -1;
}


int LCR_SetTrigOutConfig(unsigned int trigOutNum, bool invert, unsigned int rising, unsigned int falling)
/**
 * (I2C: 0x6A)
 * (USB: CMD2: 0x1A, CMD3: 0x1D)
 * This API sets the polarity, rising edge delay, and falling edge delay of the DLPC350's TRIG_OUT_1 or TRIG_OUT_2 signal.
 * The delays are compared to when the pattern is displayed on the DMD. Before executing this command,
 * stop the current pattern sequence. After executing this command, call LCR_ValidatePatLutData() API before starting the pattern sequence.
 *
 * @param   trigOutNum - I - 1 = TRIG_OUT_1; 2 = TRIG_OUT_2
 * @param   invert - I - 0 = active High signal; 1 = Active Low signal
 * @param   rising - I - rising edge delay control. Each bit adds 107.2 ns
 *                      0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xD4 = +2.68 μs, 0xD5 = +2.787 μs
 * @param   falling- I - falling edge delay control. Each bit adds 107.2 ns (This field is not applcable for TRIG_OUT_2)
 *                      0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xD4 = +2.68 μs, 0xD5 = +2.787 μs
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = invert;
    msg.text.data[3] = rising;
    msg.text.data[4] = falling;
    if(trigOutNum == 1)
        LCR_PrepWriteCmd(&msg, TRIG_OUT1_CTL);
    else if(trigOutNum==2)
        LCR_PrepWriteCmd(&msg, TRIG_OUT2_CTL);

    return LCR_SendMsg(&msg);
}

int LCR_GetTrigOutConfig(unsigned int trigOutNum, bool *pInvert,unsigned int *pRising, unsigned int *pFalling)
/**
 * (I2C: 0x6A)
 * (USB: CMD2: 0x1A, CMD3: 0x1D)
 * This API readsback the polarity, rising edge delay, and falling edge delay of the DLPC350's TRIG_OUT_1 or TRIG_OUT_2 signal.
 * The delays are compared to when the pattern is displayed on the DMD.
 *
 * @param   trigOutNum - I - 1 = TRIG_OUT_1; 2 = TRIG_OUT_2
 * @param   *pInvert - O - 0 = active High signal; 1 = Active Low signal
 * @param   *pRising - O - rising edge delay control. Each bit adds 107.2 ns
 *                      0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xD4 = +2.68 μs, 0xD5 = +2.787 μs
 * @param   *pFalling- O - falling edge delay control. Each bit adds 107.2 ns (This field is not applcable for TRIG_OUT_2)
 *                      0x00 = -20.05 μs, 0x01 = -19.9428 μs, ......0xBB=0.00 μs, ......, 0xD4 = +2.68 μs, 0xD5 = +2.787 μs
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    if(trigOutNum == 1)
        LCR_PrepReadCmd(TRIG_OUT1_CTL);
    else if(trigOutNum==2)
        LCR_PrepReadCmd(TRIG_OUT2_CTL);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pInvert = (msg.text.data[0] != 0);
        *pRising = msg.text.data[1];
        *pFalling = msg.text.data[2];
        return 0;
    }
    return -1;
}

int LCR_ValidatePatLutData(unsigned int *pStatus)
/**
 * (I2C: 0x7D)
 * (USB: CMD2: 0x1A, CMD3: 0x1A)
 * This API checks the programmed pattern display modes and indicates any invalid settings.
 * This command needs to be executed after all pattern display configurations have been completed.
 *
 * @param   *pStatus - O
 *                      BIT0 = Validity of exposure or frame period settings
 *                             1 = Selected exposure or frame period settings are invalid
 *                             0 =Selected exposure or frame period settings are valid
 *                      BIT1 = Validity of pattern numbers in lookup table (LUT)
 *                             1 = Selected pattern numbers in LUT are invalid
 *                             0 = Selected pattern numbers in LUT are valid
 *                      BIT2 = Status of Trigger Out1
 *                             1 = Warning, continuous Trigger Out1 request or overlapping black sectors
 *                             0 = Trigger Out1 settings are valid
 *                      BIT3 = Status of post sector settings
 *                             1 = Warning, post vector was not inserted prior to external triggered vector
 *                             0 = Post vector settings are valid
 *                      BIT4 = Status of frame period and exposure difference
 *                             1 = Warning, frame period or exposure difference is less than 230usec
 *                             0 = Frame period or exposure difference is valid
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepWriteCmd(&msg, LUT_VALID);    
    if(LCR_SendMsg(&msg) < 0)
        return -1;

    LCR_PrepReadCmd(LUT_VALID);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pStatus = msg.text.data[0];
        return 0;
    }
    return -1;
}


int LCR_SetTrigIn1Delay(unsigned int Delay)
/**
 * (I2C: 0x79)
 * (USB: CMD2: 0x1A, CMD3: 0x35)
 * This API sets the rising edge delay of the DLPC350's TRIG_OUT_1 signal compared to when the pattern is displayed on the DMD.
 * The polarity of TRIG_IN_1 is set in the lookup table of the pattern sequence.Before executing this command,
 * stop the current pattern sequence. After executing this command, call LCR_ValidatePatLutData() API before starting the pattern sequence.
 *
 * @param   Delay - I - rising edge delay control. 0=0ns. Each bit adds 107.2 ns
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = Delay;
    msg.text.data[3] = Delay >> 8;
    msg.text.data[4] = Delay >> 16;
    msg.text.data[5] = Delay >> 24;
    LCR_PrepWriteCmd(&msg, TRIG_IN1_DELAY);

    return LCR_SendMsg(&msg);
}

int LCR_GetTrigIn1Delay(unsigned int *pDelay)
/**
 * (I2C: 0x79)
 * (USB: CMD2: 0x1A, CMD3: 0x35)
 * This API reads back the rising edge delay of the DLPC350's TRIG_OUT_1 signal compared to when the pattern is displayed on the DMD.
 *
 * @param   *pDelay - O - rising edge delay control. 0=0ns. Each bit adds 107.2 ns
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(TRIG_IN1_DELAY);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pDelay = msg.text.data[0] | msg.text.data[1]<<8 | msg.text.data[2]<<16 | msg.text.data[3]<<24;
        return 0;
    }
    return -1;
}

int LCR_SetInvertData(bool invert)
/**
 * (I2C: 0x74)
 * (USB: CMD2: 0x1A, CMD3: 0x30)
 * This API dictates how the DLPC350 interprets a value of 0 or 1 to control mirror position for displayed patterns.
 * Before executing this command, stop the current pattern sequence. After executing this command, call
 * LCR_ValidatePatLutData() API before starting the pattern sequence.
 *
 * @param   invert - I - Pattern Display Invert Data
 *                      0 = Normal operation. A data value of 1 will flip the mirror to output light,
 *                          while a data value of 0 will flip the mirror to block light
 *                      1 = Inverted operation. A data value of 0 will flip the mirror to output light,
 *                          while a data value of 1 will flip the mirror to block light
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = invert;
    LCR_PrepWriteCmd(&msg, INVERT_DATA);

    return LCR_SendMsg(&msg);
}

int LCR_SetPWMConfig(unsigned int channel, unsigned int pulsePeriod, unsigned int dutyCycle)
/**
 * (I2C: 0x41)
 * (USB: CMD2: 0x1A, CMD3: 0x11)
 * This API sets the clock period and duty cycle of the specified PWM channel. The PWM
 * frequency and duty cycle is derived from an internal 18.67MHz clock. To calculate the desired PWM
 * period, divide the desired clock frequency from the internal 18.67Mhz clock. For example, a PWM
 * frequency of 2kHz, requires pulse period to be set to 18666667 / 2000 = 9333.
 *
 * @param   channel - I - PWM Channel Select
 *                      0 - PWM channel 0 (GPIO_0)
 *                      1 - Reserved
 *                      2 - PWM channel 2 (GPIO_2)
 *
 * @param   pulsePeriod - I - Clock Period in increments of 53.57ns. Clock Period = (value + 1) * 53.5ns
 *
 * @param   dutyCycle - I - Duty Cycle = (value + 1)% Value range is 1%-99%
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = channel;
    msg.text.data[3] = pulsePeriod;
    msg.text.data[4] = pulsePeriod >> 8;
    msg.text.data[5] = pulsePeriod >> 16;
    msg.text.data[6] = pulsePeriod >> 24;
    msg.text.data[7] = dutyCycle;

    LCR_PrepWriteCmd(&msg, PWM_SETUP);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMConfig(unsigned int channel, unsigned int *pPulsePeriod, unsigned int *pDutyCycle)
/**
 * (I2C: 0x41)
 * (USB: CMD2: 0x1A, CMD3: 0x11)
 * This API reads back the clock period and duty cycle of the specified PWM channel. The PWM
 * frequency and duty cycle is derived from an internal 18.67MHz clock. To calculate the desired PWM
 * period, divide the desired clock frequency from the internal 18.67Mhz clock. For example, a PWM
 * frequency of 2kHz, requires pulse period to be set to 18666667 / 2000 = 9333.
 *
 * @param   channel - I - PWM Channel Select
 *                      0 - PWM channel 0 (GPIO_0)
 *                      1 - Reserved
 *                      2 - PWM channel 2 (GPIO_2)
 *
 * @param   *pPulsePeriod - O - Clock Period in increments of 53.57ns. Clock Period = (value + 1) * 53.5ns
 *
 * @param   *pDutyCycle - O - Duty Cycle = (value + 1)% Value range is 1%-99%
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_SETUP, (unsigned char)channel);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pPulsePeriod = msg.text.data[1] | msg.text.data[2] << 8 | msg.text.data[3] << 16 | msg.text.data[4] << 24;
        *pDutyCycle = msg.text.data[5];
        return 0;
    }
    return -1;
}

int LCR_SetPWMEnable(unsigned int channel, bool Enable)
/**
 * (I2C: 0x40)
 * (USB: CMD2: 0x1A, CMD3: 0x10)
 * After the PWM Setup command configures the clock period and duty cycle, the PWM Enable command
 * activates the PWM signals.
 *
 * @param   channel - I - PWM Channel Select
 *                      0 - PWM channel 0 (GPIO_0)
 *                      1 - Reserved
 *                      2 - PWM channel 2 (GPIO_2)
 *
 * @param   Enable - I - PWM Channel enable 0=disable; 1=enable
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;
    unsigned char value = 0;

    if(Enable)
        value = BIT7;

    if(channel == 2)
        value |= 2;
    else if (channel != 0)
        return -1;

    msg.text.data[2] = value;
    LCR_PrepWriteCmd(&msg, PWM_ENABLE);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMEnable(unsigned int channel, bool *pEnable)
/**
 * (I2C: 0x40)
 * (USB: CMD2: 0x1A, CMD3: 0x10)
 * Reads back the enabled/disabled status of the given PWM channel.
 *
 * @param   channel - I - PWM Channel Select
 *                      0 - PWM channel 0 (GPIO_0)
 *                      1 - Reserved
 *                      2 - PWM channel 2 (GPIO_2)
 *
 * @param   *pEnable - O - PWM Channel enable 0=disable; 1=enable
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_ENABLE, (unsigned char)channel);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);       
        if(msg.text.data[0] & BIT7)
            *pEnable =  true;
        else
            *pEnable = false;

        return 0;
    }
    return -1;
}

int LCR_SetPWMCaptureConfig(unsigned int channel, bool enable, unsigned int sampleRate)
/**
 * (I2C: 0x43)
 * (USB: CMD2: 0x1A, CMD3: 0x12)
 * This API samples the specified PWM input signals and returns the PWM clock period.
 *
 * @param   channel - I - PWM Capture Port
 *                      0 - PWM input channel 0 (GPIO_5)
 *                      1 - PWM input channel 1 (GPIO_6)
 *
 * @param   enable - I - PWM Channel enable 0=disable; 1=enable
 *
 * @param   sampleRate - I - PWM Sample Rate (285 Hz to 18,666,667 Hz) - Sample Rate = Pulse Frequency / Duty Cycle
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;
    unsigned char value = 0;

    value = channel & 1;

    if(enable)
        value |= BIT7;

    msg.text.data[2] = value;
    msg.text.data[3] = sampleRate;
    msg.text.data[4] = sampleRate >> 8;
    msg.text.data[5] = sampleRate >> 16;
    msg.text.data[6] = sampleRate >> 24;
    LCR_PrepWriteCmd(&msg, PWM_CAPTURE_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetPWMCaptureConfig(unsigned int channel, bool *pEnabled, unsigned int *pSampleRate)
/**
 * (I2C: 0x43)
 * (USB: CMD2: 0x1A, CMD3: 0x12)
 * This API reads back the configuration of the specified PWM capture channel.
 *
 * @param   channel - I - PWM Capture Port
 *                      0 - PWM input channel 0 (GPIO_5)
 *                      1 - PWM input channel 1 (GPIO_6)
 *
 * @param   *pEnabled - O - PWM Channel enable 0=disable; 1=enable
 *
 * @param   *pSampleRate - O - PWM Sample Rate (285 Hz to 18,666,667 Hz) - Sample Rate = Pulse Frequency / Duty Cycle
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_CAPTURE_CONFIG, (unsigned char)channel);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        if(msg.text.data[0] & BIT7)
            *pEnabled =  true;
        else
            *pEnabled = false;

        *pSampleRate = msg.text.data[1] | msg.text.data[2] << 8 | msg.text.data[3] << 16 | msg.text.data[4] << 24;

        return 0;
    }
    return -1;
}

int LCR_PWMCaptureRead(unsigned int channel, unsigned int *pLowPeriod, unsigned int *pHighPeriod)
/**
 * (I2C: 0x4E)
 * (USB: CMD2: 0x1A, CMD3: 0x13)
 * This API returns both the number of clock cycles the signal was low and high.
 *
 * @param   channel - I - PWM Capture Port
 *                      0 - PWM input channel 0 (GPIO_5)
 *                      1 - PWM input channel 1 (GPIO_6)
 *
 * @param   *pLowPeriod - O - indicates how many samples were taken during a low signal
 *
 * @param   *pHighPeriod - O - indicates how many samples were taken during a high signal
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(PWM_CAPTURE_READ, (unsigned char)channel);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pLowPeriod = msg.text.data[1] | msg.text.data[2] << 8;
        *pHighPeriod = msg.text.data[3] | msg.text.data[4] << 8;
        return 0;
    }
    return -1;
}

int LCR_SetGPIOConfig(unsigned int pinNum, bool enAltFunc, bool altFunc1, bool dirOutput, bool outTypeOpenDrain, bool pinState)
/**
 * (I2C: 0x44)
 * (USB: CMD2: 0x1A, CMD3: 0x38)
 *
 * This API enables GPIO functionality on a specific set of DLPC350 pins. The
 * command sets their direction, output buffer type, and output state.
 *
 * @param   pinNum - I - GPIO selection. See Table 2-38 in the programmer's guide for description of available pins
 *
 * @param   enAltFunc - I - 0=disable alternative function; enable GPIO; 1=enable alternative function; disable GPIO
 *
 * @param   altFunc1 - I - must be set to false
 *
 * @param   dirOutput - I - 0=input; 1=output
 *
 * @param   outTypeOpenDrain - I - 0=Standard buffer (drives high or low); 1=open drain buffer (drives low only)
 *
 * @param   pinState - I - 0=LOW; 1=HIGH
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;
    unsigned char value = 0;

    if(enAltFunc)
        value |= BIT7;
    if(altFunc1)
        value |= BIT6;
    if(dirOutput)
        value |= BIT5;
    if(outTypeOpenDrain)
        value |= BIT4;
    if(pinState)
        value |= BIT3;

    msg.text.data[2] = pinNum;
    msg.text.data[3] = value;
    LCR_PrepWriteCmd(&msg, GPIO_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetGPIOConfig(unsigned int pinNum, bool *pEnAltFunc, bool *pAltFunc1, bool *pDirOutput, bool *pOutTypeOpenDrain, bool *pState)
/**
 * (I2C: 0x44)
 * (USB: CMD2: 0x1A, CMD3: 0x38)
 *
 * This API reads back the GPIO configuration on a specific set of DLPC350 pins. The
 * command reads back their direction, output buffer type, and  state.
 *
 * @param   pinNum - I - GPIO selection. See Table 2-38 in the programmer's guide for description of available pins
 *
 * @param   *pEnAltFunc - O - 0=disable alternative function; enable GPIO; 1=enable alternative function; disable GPIO
 *
 * @param   *pAltFunc1 - O - must be set to false
 *
 * @param   *pDirOutput - O - 0=input; 1=output
 *
 * @param   *pOutTypeOpenDrain - O - 0=Standard buffer (drives high or low); 1=open drain buffer (drives low only)
 *
 * @param   *pState - O - 0=LOW; 1=HIGH
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(GPIO_CONFIG, (unsigned char)pinNum);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pEnAltFunc = ((msg.text.data[1] & BIT7) == BIT7);
        *pAltFunc1 = ((msg.text.data[1] & BIT6) == BIT6);
        *pDirOutput = ((msg.text.data[1] & BIT5) == BIT5);
        *pOutTypeOpenDrain = ((msg.text.data[1] & BIT4) == BIT4);
        if(*pDirOutput)
            *pState = ((msg.text.data[1] & BIT3) == BIT3);
        else
            *pState = ((msg.text.data[1] & BIT2) == BIT2);
        return 0;
    }
    return -1;
}

int LCR_SetGeneralPurposeClockOutFreq(unsigned int clkId, bool enable, unsigned int clkDivider)
/**
 * (I2C: 0x48)
 * (USB: CMD2: 0x08, CMD3: 0x07)
 *
 * DLPC350 supports two pins with clock output capabilities: GPIO_11 and GPIO_12.
 * This API enables the clock output functionality and sets the clock frequency.
 *
 * @param   clkId - I - Clock selection. 1=GPIO_11; 2=GPIO_12
 *
 * @param   enable - I - 0=disable clock functionality on selected pin; 1=enable clock functionality on selected pin
 *
 * @param   clkDivider - I - Allowed values in the range of 2 to 127. Output frequency = 96MHz / (Clock Divider)
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = clkId;
    msg.text.data[3] = enable;
    msg.text.data[4] = clkDivider;
    LCR_PrepWriteCmd(&msg, GPCLK_CONFIG);

    return LCR_SendMsg(&msg);
}

int LCR_GetGeneralPurposeClockOutFreq(unsigned int clkId, bool *pEnabled, unsigned int *pClkDivider)
/**
 * (I2C: 0x48)
 * (USB: CMD2: 0x08, CMD3: 0x07)
 *
 * DLPC350 supports two pins with clock output capabilities: GPIO_11 and GPIO_12.
 * This API reads back the clock output enabled status and the clock frequency.
 *
 * @param   clkId - I - Clock selection. 1=GPIO_11; 2=GPIO_12
 *
 * @param   *pEnabled - O - 0=disable clock functionality on selected pin; 1=enable clock functionality on selected pin
 *
 * @param   *pClkDivider - O - Allowed values in the range of 2 to 127. Output frequency = 96MHz / (Clock Divider)
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmdWithParam(GPCLK_CONFIG, (unsigned char)clkId);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pEnabled = (msg.text.data[0] != 0);
        *pClkDivider = msg.text.data[1];
        return 0;
    }
    return -1;
}

int LCR_SetLEDPWMInvert(bool invert)
/**
 * (I2C: 0x0B)
 * (USB: CMD2: 0x1A, CMD3: 0x05)
 *
 * This API sets the polarity of all LED PWM signals. This API must be called before powering up the LED drivers.
 *
 * @param   invert - I - 0 = Normal polarity, PWM 0 value corresponds to no current while PWM 255 value corresponds to maximum current
 *                       1 = Inverted polarity. PWM 0 value corresponds to maximum current while PWM 255 value corresponds to no current.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = invert;
    LCR_PrepWriteCmd(&msg, PWM_INVERT);

    return LCR_SendMsg(&msg);
}

int LCR_GetLEDPWMInvert(bool *inverted)
/**
 * (I2C: 0x0B)
 * (USB: CMD2: 0x1A, CMD3: 0x05)
 *
 * This API reads the polarity of all LED PWM signals.
 *
 * @param   invert - O - 0 = Normal polarity, PWM 0 value corresponds to no current while PWM 255 value corresponds to maximum current
 *                       1 = Inverted polarity. PWM 0 value corresponds to maximum current while PWM 255 value corresponds to no current.
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepReadCmd(PWM_INVERT);

    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *inverted = (msg.text.data[0] != 0);
        return 0;
    }
    return -1;
}

int LCR_MemRead(unsigned int addr, unsigned int *readWord)
/**
 *
 * This API reads back the content at a specified memory location from the controller.
 *
 * @param   addr - I - address from which to read contents
 *
 * @param   readWord - O - 32-bit word read from the given address
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    LCR_PrepMemReadCmd(addr);
    if(LCR_Read() > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *readWord = msg.text.data[0] | msg.text.data[1] << 8 | msg.text.data[2] << 16 | msg.text.data[3] << 24;
        //*readWord = msg.text.data[3] | msg.text.data[2] << 8 | msg.text.data[1] << 16 | msg.text.data[0] << 24; //MSB first
        return 0;
    }
    return -1;
}

int LCR_MemWrite(unsigned int addr, unsigned int data)
/**
 *
 * This API writes the given content at a specified memory location from the controller.
 *
 * @param   addr - I - address to write to
 *
 * @param   data - I - 32-bit word to be written at given address
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    hidMessageStruct msg;

    msg.text.data[2] = 0; //absolute write
    msg.text.data[6] = addr >> 24; 
    msg.text.data[5] = addr >> 16;
    msg.text.data[4] = addr >> 8;
    msg.text.data[3] = addr;  //LSB first
    msg.text.data[10] = data >> 24; 
    msg.text.data[9] = data >> 16;
    msg.text.data[8] = data >> 8;
    msg.text.data[7] = data;  //LSB first
    LCR_PrepWriteCmd(&msg, MEM_CONTROL);

    return LCR_SendMsg(&msg);
}

 int LCR_MeasureSplashLoadTiming(unsigned int startIndex, unsigned int numSplash)
 /**
  * This API instructs the controller to measure the load time for the image(s) stored starting at specified index.
  * The result of measuement can be read back using the LCR_ReadSplashLoadTiming() API.
  *
  * @param   startIndex - I - index of the first image whose load time is to be measured
  *
  * @param   numSplash - I - number of images for which load time is to be measured.
  *
  * @return  >=0 = PASS    <BR>
  *          <0 = FAIL  <BR>
  *
  */
 {
     hidMessageStruct msg;

     msg.text.data[2] = startIndex;
     msg.text.data[3] = numSplash;
     LCR_PrepWriteCmd(&msg, SPLASH_LOAD_TIMING);

     return LCR_SendMsg(&msg);
 }

 int LCR_ReadSplashLoadTiming(unsigned int *pTimingData)
 /**
  * This API reads back the mesasured load time for the image specified by LCR_MeasureSplashLoadTiming() API.
  *
  * @param   *pTimingData - I - time taken to load the specified image in milliseconds = value/18667.
  *
  * @return  >=0 = PASS    <BR>
  *          <0 = FAIL  <BR>
  *
  */
 {
     hidMessageStruct msg;

     LCR_PrepReadCmd(SPLASH_LOAD_TIMING);

     if(LCR_Read() > 0)
     {
         memcpy(&msg, InputBuffer, 65);
         *pTimingData = (msg.text.data[0] | msg.text.data[1] << 8 | msg.text.data[2] << 16 | msg.text.data[3] << 24);
         return 0;
     }
     return -1;
 }
