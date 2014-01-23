LightCrafter4500
================

*DLP LightCrafter 4500 Command Line for Windows*

Source code to create a simple command line control program for the TI DLP LightCrafter 4500.  Using the HDMI input as the image source, this source switches into a 180Hz 7-bit per plane with various options to control the LEDs.  It can be easily extended to initialize the device into other modes.

    usage: LCr4500Init --color [RGB|R|B|G|W] --mode [video|structuredlight|reset|powerdown|powerup|noChange] 
      --status --version
       
       no parameters is same as --color W --mode structuredlight
       parameters can be abbreviated to a single character, unless first character options are not unique
       
Examples:

    LCr4500Init --color rgb --mode video  
Same state as if the device just powered on (video processing RGB output).

    LCr4500Init --color r --mode stucturedlight  
180Hz refresh, each 7-bit color plane is interpreted as a separate image.  Planes are presented in the sequence GRB.  Only use the Red output LED.

    LCr4500Init --color w --mode stucturedlight  
180Hz refresh, each 7-bit color plane is interpreted as a separate image.  Turn on all three LEDs for each color plane.




    
    
