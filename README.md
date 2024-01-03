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

Go to path: `Source\SDK_WIN\pdk_jacinto_09_01_00_22\packages\ti\build`,

Build pdk libs,

```bat
gmake pdk_libs CORE=mcu1_0 BOARD=j721e_evm -j30 -s
```

To build all,

```bat
gmake all CORE=mcu1_0 BOARD=j721e_evm -j30 -s
```

Build the SBL, go to path: `Source\SDK_WIN\pdk_jacinto_09_01_00_22\packages\ti\boot\sbl\build`

```bat
@rem not sure why it will compile other board
gmake CORE=mcu1_0 BOARD=j721e_evm -j30 -s
```

Use this instead,

```bat
gmake all COMP=sbl CORE=mcu1_0 BOARD=j721e_evm SOC=j721e -j30 -s
```
