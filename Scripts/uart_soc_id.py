#!/usr/bin/env python3

import binascii
import struct
import sys

filename = sys.argv[1]

with open(filename, "r", encoding="UTF-8") as fp:
    lines = fp.readlines()

bin_arr = [binascii.unhexlify(x.rstrip()) for x in lines]
bin_str = b"".join(bin_arr)
# typedef struct __attribute__((packed, aligned(4)))
# {
#     uint8_t             subBlockId;
#     uint8_t             size;
#     uint8_t             fixed[2];
#     uint8_t             devName[SOCID_SECINFO_DEVNAME_LEN];
#     uint32_t            devType;
#     uint32_t            dmscVersion;
#     uint32_t            r5Version;
# }
# SOCID_PubInfo_t;
pubInfoStr = "BB2B12B4B4B4B"

# /* Secure Info applicable only for EMU/HS device types */
# typedef struct __attribute__((packed, aligned(4)))
# {
#     uint8_t             subBlockId;
#     uint8_t             size;
#     uint16_t            secPrime;
#     uint16_t            keyRevision;
#     uint16_t            keyCount;
#     uint32_t            tiRootKeyHash[SOCID_SECINFO_ROOTKEYHASH_LEN];
#     uint32_t            custRootKeyHash[SOCID_SECINFO_ROOTKEYHASH_LEN];
#     uint32_t            uniqueID[SOCID_SECINFO_UNIQUEID_LEN];
# }
# SOCID_SecInfo_t;
secInfoStr = "BBHHH64B64B32B"

# /* SOC ID definition */
# typedef struct __attribute__((packed, aligned(4)))
# {
#     uint32_t            numBlocks;
#     SOCID_PubInfo_t     pubInfo;
#     SOCID_SecInfo_t     secInfo;
# } SOCID_Obj;

(numBlocks,) = struct.unpack("I", bin_str[0:4])
pubROMInfo = struct.unpack(pubInfoStr, bin_str[4:32])
secROMInfo = struct.unpack(secInfoStr, bin_str[32:200]) if numBlocks > 1 else None

print("-----------------------")
print("SoC ID Header Info:")
print("-----------------------")
print("NumBlocks :", numBlocks)
print("-----------------------")
print("SoC ID Public ROM Info:")
print("-----------------------")
print("SubBlockId :", pubROMInfo[0])
print("SubBlockSize :", pubROMInfo[1])
tmpList = list(pubROMInfo[4:15])
hexList = [hex(i) for i in tmpList]
deviceName = "".join(chr(int(c, 16)) for c in hexList[0:])
print("DeviceName :", deviceName)

tmpList = list(pubROMInfo[16:20])
hexList = [hex(i) for i in tmpList]
deviceType = "".join(chr(int(c, 16)) for c in hexList[0:])
print("DeviceType :", deviceType)

dmscROMVer = list(pubROMInfo[20:24])
dmscROMVer.reverse()
print("DMSC ROM Version :", dmscROMVer)

r5ROMVer = list(pubROMInfo[24:28])
r5ROMVer.reverse()
print("R5 ROM Version :", r5ROMVer)

if secROMInfo is not None:
    print("-----------------------")
    print("SoC ID Secure ROM Info:")
    print("-----------------------")
    print("Sec SubBlockId :", secROMInfo[0])
    print("Sec SubBlockSize :", secROMInfo[1])
    print("Sec Prime :", secROMInfo[2])
    print("Sec Key Revision :", secROMInfo[3])
    print("Sec Key Count :", secROMInfo[4])
    tmpList = list(secROMInfo[5:69])
    tiMPKHash = "".join("{:02x}".format(x) for x in tmpList)
    print("Sec TI MPK Hash :", tiMPKHash)
    tmpList = list(secROMInfo[69:133])
    custMPKHash = "".join("{:02x}".format(x) for x in tmpList)
    print("Sec Cust MPK Hash :", custMPKHash)
    tmpList = list(secROMInfo[133:167])
    uID = "".join("{:02x}".format(x) for x in tmpList)
    print("Sec Unique ID :", uID)
