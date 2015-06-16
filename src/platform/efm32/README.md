elua on EFM32 STK3700
==============================

* Get the source code from github.
  EFM32 supported source code located at https://github.com/MarkDing/elua/tree/efm32. Get source code using git clone command. 
```
  $ git clone https://github.com/MarkDing/elua.git
  $ git checkout efm32
```

* Import EFM32 project files into Simplicity Studio.
  Open Simplicity IDE, right click on Project Explorer, select Import->MCU Project, choose elua_EFM32_STK3700.slsproj in elua root directory. 

* Copy link file into workspace directory.
  We need customized link file, it locates at src\platform\efm32\efm32.ld, copy it into Simplicity STudio workspace directory. It normally located at C drive, "SimplicityStudio\v3_workspace\elua_EFM32_STK3700" directory. 

* Build the project
  Build the project in Simplicity Studio and download it into STK3700 board.

* Using serial port terminal tool
 Open Tera Term, we can see elua prompt message displayed in the windows. 
