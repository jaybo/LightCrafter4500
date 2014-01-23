// LCr4500Init.cpp : Command line application for TI LightCrafter 4500 DLP.
//

#include <stdlib.h>
#include <stdio.h>
#include <wchar.h>
#include "stdafx.h"
#include "usb.h"
#include "API.h"
#include "LCr4500Init.h"
#include "getopt.h"

#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 1

enum LCrCommand {
    CMD_STATUS,						// Read status only
    CMD_RESET,						// Reboot, wait 10 seconds, unit is automatically in "video" mode.
    CMD_STANDBY,					// Power down, LEDs off.
    CMD_NOT_STANDBY,				// Power up
    CMD_MODE_STRUCTURED_LIGHT,		// 912x1140 display ONLY.  No scaling, no color processing.  We use this mode.
    CMD_MODE_VIDEO					// Normal video output, with scaling and color processing.  We use this for testing only.
} ;

static struct {
    LCrCommand Command;
    int ShowVersion;
    bool ShowStatus;
    bool RedOnly;
    bool GreenOnly;
    bool BlueOnly;
    bool Grayscale;					// All LEDs on for each color channel	 
    bool RGB;
} GlobalOptions;

const bool UseGetchar = false;      // wait for carriage return after running command

int _tmain(int argc, _TCHAR* argv[])
{
    int c;

    GlobalOptions.Grayscale = true;
    GlobalOptions.ShowStatus = false;
    GlobalOptions.ShowVersion = false;
    GlobalOptions.Command = CMD_MODE_STRUCTURED_LIGHT;

    while (1)
    {
        static struct option long_options[] =
        {
            { _T("mode"), ARG_REQ, 0, _T('m') },
            { _T("color"), ARG_REQ, 0, _T('c') },
            { _T("version"), ARG_NONE, 0, _T('v') },
            { _T("status"), ARG_NONE, 0, _T('s') },
            { _T("help"), ARG_NONE, 0, _T('h') },
            { _T("?"), ARG_NONE, 0, _T('?') },
            { ARG_NULL, ARG_NULL, ARG_NULL, ARG_NULL }
        };

        int option_index = 0;
        c = getopt_long(argc, argv, _T("m:c:vs?h"), long_options, &option_index);

        // Check for end of operation or error
        if (c == -1)
            break;

        if (optarg)
        {
            _wcslwr (optarg);
        }
        // Handle options
        switch (c)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (long_options[option_index].flag != 0)
                break;
            _tprintf(_T("option %s"), long_options[option_index].name);
            if (optarg)
                _tprintf(_T(" with arg %s"), optarg);
            _tprintf(_T("\n"));
            break;

        case _T('c'):
            if (optarg)
            {
                if (wcscmp(optarg, _T("rgb")) == 0)			// Like standard video, but with no scaling or color conversion
                {
                    GlobalOptions.RGB = true;
                    GlobalOptions.Grayscale = false;
                }
                else if (wcscmp(optarg, _T("w")) == 0)		// White (grayscale) at 180Hz
                {
                    GlobalOptions.Grayscale = true;
                }
                else if (wcscmp(optarg, _T("r")) == 0)		// Only use RED
                {
                    GlobalOptions.RedOnly = true;
                    GlobalOptions.Grayscale = false;
                }
                else if (wcscmp(optarg, _T("g")) == 0)		// Only use Green
                {
                    GlobalOptions.GreenOnly = true;
                    GlobalOptions.Grayscale = false;
                }
                else if (wcscmp(optarg, _T("b")) == 0)		// Only use Blue
                {
                    GlobalOptions.BlueOnly = true;
                    GlobalOptions.Grayscale = false;
                }
            }
            //_tprintf(_T("option -c with value `%s'\n"), optarg);
            break;

        case _T('s'):		// Status
            GlobalOptions.ShowStatus = true;
            break;

        case _T('v'):		// Version
            GlobalOptions.ShowVersion = true;
            break;

        case _T('m'):
            if (optarg){
                if (wcscmp(optarg, _T("powerdown")) == 0)			
                {
                    GlobalOptions.Command = CMD_STANDBY;
                }
                else if (wcscmp(optarg, _T("powerup")) == 0)
                {
                    GlobalOptions.Command = CMD_NOT_STANDBY;
                }
                else if ((optarg[0]) == _T('r'))
                {
                    GlobalOptions.Command = CMD_RESET;
                }
                else if ((optarg[0]) == _T('v'))			// "video"
                {
                    GlobalOptions.Command = CMD_MODE_VIDEO;
                }
                else if ((optarg[0]) == _T('s'))	// "structured light" or "sl"
                {
                    GlobalOptions.Command = CMD_MODE_STRUCTURED_LIGHT;
                }
                else if ((optarg[0]) == _T('n'))	// "NO CHANGE, READ STATUS ONLY"
                {
                    GlobalOptions.Command = CMD_STATUS;
                }
            }
            break;

        case '?':
        case 'h':
        default:
            _tprintf(_T("\nusage: --color [RGB|R|B|G|W] --mode [video|structuredlight|reset|powerdown|powerup|noChange] --status --version \n"));
            _tprintf(_T("       no parameters is same as --color W --mode structuredlight\n"));
            _tprintf(_T("       parameters can be abbreviated to a single character, unless first character options are not unique"));
            return -1;
            break;

        }
    }

    if (LCr_Connect())
    {
        switch (GlobalOptions.Command)
        {
        case CMD_RESET:
            LCr_Reset();
            _sleep(10000);						// Reboot takes about 5 seconds
            break;
        case CMD_STANDBY:
            LCr_Standby(true);					// powerdown
            break;
        case CMD_NOT_STANDBY:
            LCr_Standby(false);					// powerup
            break;
        case CMD_STATUS:						// just display status
            // nothing to do
            break;
        case CMD_MODE_STRUCTURED_LIGHT:			// 180Hz mode
            LCr_StructuredLightMode();
            break;
        case CMD_MODE_VIDEO:					// 60Hz mode
            LCr_StandardVideoMode();
            break;
        }

        if (GlobalOptions.ShowStatus)
        {
            _sleep(1000);
            LCr_Status();
        }

        if (UseGetchar) getchar();

        return 0;
    }
    return 1;
}


bool LCr_Connect()
{
    char versionStr[255];
    unsigned int API_ver, App_ver, SWConfig_ver, SeqConfig_ver;
    unsigned int FW_ver;
    bool SLmode = 0;

    if (USB_IsConnected())
        USB_Close();
    if (USB_Open() != 0)
    {
        printf("Error, cannot connect to LightCrafter 4500\n");
        return false;
    }

    // Display App Version #
    sprintf_s(versionStr, "DLP LightCrafter 4500 Init - %d.%d\n", APP_VERSION_MAJOR, APP_VERSION_MINOR);
    printf(versionStr);

    if (USB_IsConnected())
    {
        if (GlobalOptions.ShowVersion)
        {
            if (LCR_GetVersion(&App_ver, &API_ver, &SWConfig_ver, &SeqConfig_ver) == 0)
            {
                sprintf_s(versionStr, "APIVer:    %d.%d.%d\n", (API_ver >> 24), ((API_ver << 8) >> 24), ((API_ver << 16) >> 16));
                printf(versionStr);
                sprintf_s(versionStr, "AppVer:    %d.%d.%d\n", (App_ver >> 24), ((App_ver << 8) >> 24), ((App_ver << 16) >> 16));
                printf(versionStr);
                sprintf_s(versionStr, "SWConfig:  %d.%d.%d\n", (SWConfig_ver >> 24), ((SWConfig_ver << 8) >> 24), ((SWConfig_ver << 16) >> 16));
                printf(versionStr);
                sprintf_s(versionStr, "SeqConfig: %d.%d.%d\n", (SeqConfig_ver >> 24), ((SeqConfig_ver << 8) >> 24), ((SeqConfig_ver << 16) >> 16));
                printf(versionStr);
            }

            if (LCR_MemRead(0xF902C000, &FW_ver) == 0)
            {
                FW_ver &= 0xFFFFFF;
                sprintf_s(versionStr, "Firmware:  %d.%d.%d\n", (FW_ver >> 16), ((FW_ver << 16) >> 24), ((FW_ver << 24) >> 24));
                printf(versionStr);

                // When GUI is first opened, check if old firmware is present & prompt user to upload new version if it is
                /*if (FirstConnection && (FW_ver < RELEASE_FW_VERSION))
                {
                showError("WARNING: Old version of Firmware detected.\n\nDownload the latest release from http://www.ti.com/tool/dlpr350.");
                FirstConnection = FALSE;
                }*/
            }
        }
        return true;
    }
    return false;
}


bool LCr_Status()
{
    unsigned char HWStatus, SysStatus, MainStatus;

    if (LCR_GetStatus(&HWStatus, &SysStatus, &MainStatus) == 0)
    {
        printf("Status ------------------------- \n");
        if ((HWStatus & BIT0) == BIT0) { //Init Done
            printf("Init Done\n");
        }
        if ((HWStatus & BIT3) == BIT3) { //Forced Swap
            printf("Forced Swap\n");
        }
        if ((HWStatus & BIT6) == BIT6) { //Sequence Abort
            printf("Sequence Abort\n");
        }
        if ((HWStatus & BIT2) == BIT2) { // DRC Error
            printf("DRC Error\n");
        }
        if ((HWStatus & BIT7) == BIT7) { // Sequence Error
            printf("Sequence Error\n");
        }

        if ((MainStatus & BIT0) == BIT0) { //DMD Parked
            printf("DMD Parked\n");
        }
        if ((MainStatus & BIT1) == BIT1){ // Sequence Running
            printf("Sequence Running\n");
        }
        if ((MainStatus & BIT2) == BIT2){ //Buffer frozen
            printf("Buffer frozen\n");
        }
        return true;
    }
    return false;
}

void LCr_Reset()
{
    LCR_SoftwareReset();
}

void LCr_Standby(bool powerdown)
{
    LCR_SetPowerMode(powerdown);
}

void LCr_StandardVideoMode()
{
    printf("LCr_StandardVideoMode\n");
    LCR_SetPowerMode(0);
    LCR_SetMode(0);
}

#define FRAME_PERIOD 5000		// microseconds
#define EXPOSURE_PERIOD 5000	// microseconds

struct LutEntry {
    int TrigType;
    int PatNum;
    int BitDepth;
    int LEDSelect;
    bool InvertPat;
    bool InsertBlack;
    bool BufSwap;
    bool trigOutPrev;
};


void LCr_StructuredLightMode()
{
    printf("LCr_StructuredLightMode\n");

    LutEntry LutEntries[] = {
        {
            1,		// int TrigType
            0,		// int PatNum
            7,		// int BitDepth
            (GlobalOptions.RGB || GlobalOptions.GreenOnly) ? 2 : GlobalOptions.Grayscale ? 7 : 0,		// int LEDSelect
            false,	// bool InvertPat
            true,	// bool InsertBlack
            false,	// bool BufSwap
            false	// bool trigOutPrev))
        },
        {
            3,		// int TrigType
            1,		// int PatNum
            7,		// int BitDepth
            (GlobalOptions.RGB || GlobalOptions.RedOnly) ? 1 : GlobalOptions.Grayscale ? 7 : 0,		// int LEDSelect
            false,	// bool InvertPat
            true,	// bool InsertBlack
            false,	// bool BufSwap
            false	// bool trigOutPrev))
        },
        {
            3,		// int TrigType
            2,		// int PatNum
            7,		// int BitDepth
            (GlobalOptions.RGB || GlobalOptions.BlueOnly) ? 4 : GlobalOptions.Grayscale ? 7 : 0,		// int LEDSelect
            false,	// bool InvertPat
            true,	// bool InsertBlack
            false,	// bool BufSwap
            false	// bool trigOutPrev))

        }
    };

    int NumberOfLutEntries = (sizeof(LutEntries) / sizeof(LutEntry));

    if (LCR_SetMode(true) < 0) 					// Pattern display mode
    {
        printf("Error LCR_SetMode\n");
        return;
    }

    if (LCR_PatternDisplay(0) < 0)				// Stop Pattern Display
    {
        printf("Error LCR_PatternDisplay\n");
        return;
    }

    if (LCR_SetInputSource(0, 1) < 0)			// FPD-Link, 24 bit
    {
        printf("Error LCR_SetInputSource\n");
        return;
    }

    if (LCR_SetPatternDisplayMode(true) < 0)	// FPD-Link
    {
        printf("Error LCR_SetPatternDisplayMode\n");
        return;
    }

    LCR_ClearPatLut();

    for (int j = 0; j < NumberOfLutEntries; j++)
    {
        if (LCR_AddToPatLut(
            LutEntries[j].TrigType,
            LutEntries[j].PatNum,
            LutEntries[j].BitDepth,
            LutEntries[j].LEDSelect,
            LutEntries[j].InvertPat,
            LutEntries[j].InsertBlack,
            LutEntries[j].BufSwap,
            LutEntries[j].trigOutPrev
            ) < 0)
        {
            printf("Error LCR_AddToPatLut for index: %d\n", j);
            return;
        }
    }

    if (LCR_SetPatternConfig(
        NumberOfLutEntries /*numLutEntries*/,
        1 /*repeat*/, 
        NumberOfLutEntries /*numPatsForTrigOut2*/,
        1 /*numSplashLutEntries*/) < 0)
    {
        printf("Error LCR_SetPatternConfig\n");
        return;
    }

    if (LCR_SetExposure_FramePeriod(EXPOSURE_PERIOD /*unsigned int exposurePeriod*/, FRAME_PERIOD /*unsigned int framePeriod*/) < 0)
    {
        printf("Error LCR_SetExposure_FramePeriod\n");
        return;
    }

    if (LCR_SetPatternTriggerMode(0) < 0)		// VSync triggers pattern
    {
        printf("Error Sending trigger Mode\n");
        return;
    }

    if (LCR_SendPatLut() < 0)
    {
        printf("Error LCR_SendPatLut\n");
        return;
    }

    unsigned char splashLut[64];
    splashLut[0] = 0;	// No clue what I'm doing here...

    if (LCR_SendSplashLut(&splashLut[0], 1) < 0)
    {
    	printf("Error Sending Image LUT");
    	return;
    }

    unsigned int status;
    if (LCR_ValidatePatLutData(&status) < 0)
    {
        printf("Error validating LUT data\n");
        return;
    }
    
    if (LCR_PatternDisplay(2) < 0) //Start pattern display
    {
        printf("Error starting pattern display\n");
        return;
    }

}
