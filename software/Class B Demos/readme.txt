
This is a readme file for the PIC32MX ClassB demo application.


1.  Open the ClassB.mcw workspace that contains the demo project.

2.  The demo is supposed to run on PIC32MX795F512L part.
    Use this part or change the processor type in the project configuration.
    
3.  Notice that the project uses a modified linker script elf32pic32mx.ld.
    This script is manually updated for the PIC32MX795F512L. If you use a part with a
    different memory map either update the linker script or remove the elf32pic32mx.ld
    from the project.
    Note: the elf32pic32mx.ld is useful only for the absolute placement of the
    PC test functions.
    The project will run just fine if the PC test functions are placed at linker default
    addresses. 

4.  All the necessary files are already part of the project.
    The ClassB_demo.c is the main file for the demo.
    It calls all the ClassB test functions in turn using meaningful
    values for the parameters.
    
5.  A standard demo board, such as the Explorer 16 can be used.    

6.  Select a debugger of your choice to connect to the demo processor.
    Build the project and program it on the part.
    Set a breakpoint at the end of the demo (ClassB_demo.c:: line 354) and then run.
    Alternatively step through the demo application and see how each ClassB test function
    is called in turn.
    
7.  After hitting the end of demo breakpoint the variable testFlag should have
    all the bit fields set (this indicates that all the tests have passed successfully).

8.  Notice that the last 2 tests, the clock tests have special hardware requirements:
    - the 32.768 KHz chrystal should be connected to the SOSC input.
    - A 50 HZ reference signal should be fed to the IC1 input.
    
9.  The Microchip ClassB Safety Software Library AN1229 containing the
    detailed API description is included in the
    "instal_path\Microchip\ClassB\documentation" directory.
    
    
