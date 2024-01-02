# README

An example output of the SOC ID with the out of box board,

```bat
>python soc_id.py id.txt
-----------------------
SoC ID Header Info:
-----------------------
NumBlocks : 1
-----------------------
SoC ID Public ROM Info:
-----------------------
SubBlockId : 1
SubBlockSize : 26
DeviceName : j7es
DeviceType : GP
DMSC ROM Version : [0, 1, 1, 1]
R5 ROM Version : [0, 1, 1, 1]
```

## Build

Build all libs,

```bat
gmake all_libs CORE=mcu1_0 BOARD=j721e_evm -j30 -s
```

```bat
gmake all CORE=mcu1_0 BOARD=j721e_evm -j30 -s
```
