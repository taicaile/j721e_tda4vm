#!/bin/bash

##################################################################
# File: gen_keywr_cert.sh
# 
# Description: Script to generate x509 certificate for key wirter, 
#              which has encrypted customer key information.
# 
# (c) Texas Instruments 2020, All rights reserved.
##################################################################

source gen_keywr_cert_helpers.sh

# OTP Keywriter Version 00.00.02.00
KEYWRITER_VERSION="00000200"
ENABLE_VAL="5A"
DISABLE_VAL="A5"

HELPTXT="
	./gen_keywr_cert.sh - creates a x509 certificate from the keys input to this script.
	-a | --aes256
		Path to file
		256 bit (symmetric key)
		AES-256 key to be used (supposed to be randomly generated by customer)
		If not specified, a random 256 bit key is chosen by the script
	-b | --bmpk
		Path to file
		4096 bit (RSA private key, pem format)
		BMPK key to be used (not necessary, in case customer doesn't want to program
		BMPK, BMEK keys)
	-b-wp
	-b-rp
	-b-ovrd
	--bmek
		Path to file
		256 bit (symmetric key, binary file)
		BMEK key to be used (not necessary, in case customer doesn't want to program
		BMPK, BMEK keys)
	--bmek-wp
	--bmek-rp
	--bmek-ovrd
	-c | --cert
		Path to file
		Path to save final certificate
		Default is '../x509cert/final_certificate.cert'
	-g | --gen
		Generate aes256 key, bmpk, bmek, smpk, smek in keys/ folder
	-h
		Print Help Text
	--msv 
		20 bit hexadecimal value
		Defaults to 0x0000
		MSV - Model Specific Value
	--msv-wp
	--msv-rp
	--msv-ovrd
	--keycnt
		Either 1 or 2
		Key Count
	--keycnt-wp
	--keycnt-rp
	--keycnt-ovrd
	--keyrev
		Either 1 or 2
		Note: If Key Rev is 2, BMPKH and BMEK are used for sec boot
		Key Revision
	--keyrev-wp
	--keyrev-rp
	--keyrev-ovrd
	-s | --smpk 
		Path to file
		4096 bit (RSA private key, pem format)
		SMPK key to be used
	-s-wp
	-s-rp
	-s-ovrd
	--smek 
		Path to file
		256 bit (symmetric key, binary file)
		SMEK key to be used
	--smek-wp
	--smek-rp
	--smek-ovrd
	--sr-bcfg 
		Supports 1 to 64
		SWREV_SEC_BRDCFG - Software Revision Secure Board Config
	--sr-bcfg-wp
	--sr-bcfg-rp
	--sr-bcfg-ovrd
	--sr-sbl
		Supports 1 to 48
		SWREV_SBL -  Software Revision SBL
	--sr-sbl-wp
	--sr-sbl-rp
	--sr-sbl-ovrd
	--sr-sysfw
		Supports 1 to 48
		SWREV_SYSFW - Software Revision SYSFW
	--sr-sysfw-wp
	--sr-sysfw-rp
	--sr-sysfw-ovrd
	--ext-otp
		Path to file
		Should be exactly 1024 bits
	--ext-otp-indx
		0 to 1023
		Index to start, with refernce to 1024 bits 
	--ext-otp-size
		1 to 1024
		Size of ext_otp_data in 1024 bits, starting at the index mentioned
	--ext-otp-wprp
		128 bit value
		Each bit (0:63 - rp, 64:127 - wp) represents individual row
	-t | --tifek (mandatory)
		4096 bit (RSA publick key, pem format)
		TI provided Forward Encryption Key
	--mpk-opt
		10 bit value
		Defaults to 0
		Public Key options
	--mek-opt
		5 bit value
		Defaults to 0
		Symmetric Key options


	Example Usage:
	
	# To generate keys
	./gen_keywr_cert.sh -g

	# To generate certificate
	./gen_keywr_cert.sh -s keys/smpk.pem --smek keys/smek.key -b keys/bmpk.pem --bmek keys/bmek.key -t ti_fek_public.pem -a keys/aes256.key
	
	./gen_keywr_cert.sh -s keys/smpk.pem --smek keys/smek.key -t ti_fek_public.pem -a keys/aes256.key --msv 0xC0FFE --keycnt 1 --keyrev 1 --sr-bcfg 16 --sr-sbl 16 --sr-sysfw 16 --ext-otp ext_otp_data.bin --ext-otp-indx 1 --ext-otp-size 3
"

## HELP ------------------------------------------------------------
if [ "$#" -lt 1 ]; then
	echo "$HELPTXT"
	exit
fi

######################################################################################
# DECLARE TEMPORARY VARIABLES
######################################################################################

declare -A smpk_info
declare -A smek_info
declare -A bmpk_info
declare -A bmek_info
declare -A msv_info
declare -A key_cnt_info
declare -A key_rev_info
declare -A swrev_sysfw_info
declare -A swrev_sbl_info
declare -A swrev_sec_brdcfg_info
declare -A extotp_info
declare -A mpk_opt_info
declare -A mek_opt_info

declare -A tifek_info
declare -A aes256key_info

declare -A primary_cert_info
declare -A secondary_cert_info

declare -A output_info

smpk_info[flag]="no"
smek_info[flag]="no"
bmpk_info[flag]="no"
bmek_info[flag]="no"
# 20 bits
msv_info[val]="00000000"
msv_info[flag]="no"
# 8 bits
key_cnt_info[val]="01"
# 8 bits
key_rev_info[val]="01"
# 48 bits
swrev_sysfw_info[val]="000000000000"
# 48 bits
swrev_sbl_info[val]="000000000000"
# 64 bits
swrev_sec_brdcfg_info[val]="0000000000000000"
# 1024 bits
extotp_info[val]="0"
extotp_info[index]="0"
extotp_info[size]="0"
extotp_info[wprp]="00000000000000000000000000000000"

mpk_opt_info[val]="0000"
mek_opt_info[val]="00"

tifek_info[flag]="no"
aes256key_info[flag]="no"

secondary_cert_info[flag]="no"

primary_cert_info[config]="configs/prim_cert_config.txt"
secondary_cert_info[config]="configs/sec_cert_config.txt"

secondary_cert_info[file]="secondary_cert.bin"
primary_cert_info[file]="primary_cert.bin"

output_info[hash_csv]="verify_hash.csv"
output_info[certificate]="../x509cert/final_certificate.bin"

## PARSE ARGUMENTS -------------------------------------------------
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -a|--aes256)
	if [[ "${aes256key_info[flag]}" == "no" ]]; then
		aes256key_info[flag]="yes"
		aes256key_info[file]="$2"
	else
		echo "ERR: AES256 key provided multiple times"
		exit
	fi
    shift # past argument
    shift # past value
    ;;
	--ext-otp)
		if [ -f "$2" ]; then
			extotp_info[flag]="yes"
			extotp_info[file]="$2"
		else
			echo "ERR: $2 doesn't exist. Consider using construct_ext_otp_data.sh"
			exit
		fi
	shift # past argument
    shift # past value
    ;;
	--ext-otp-indx)
		extotp_info[index]="$2"
	shift # past argument
    shift # past value
    ;;
	--ext-otp-size)
		extotp_info[size]="$2"
	shift # past argument
    shift # past value
    ;;
	--ext-otp-wprp)
		extotp_info[wprp]="$2"
		parse_validate_extotp_wprp "$2"
	shift
	shift
	;;
	-t|--tifek)
	if [[ "${tifek_info[flag]}" == "no" ]]; then
		tifek_info[flag]="yes"
		tifek_info[file]="$2"
	else
		echo "ERR: TI FEK provided multiple times"
		exit
	fi
    shift # past argument
    shift # past value
    ;;
	-s|--smpk)
	if [[ "${smpk_info[flag]}" == "no" ]]; then
		smpk_info[flag]="yes"
		smpk_info[file]="$2"
	else
		echo "ERR: SMPK provided multiple times"
		exit
	fi
    shift # past argument
    shift # past value
    ;;
	-s-wp)
		smpk_info[wp]="yes"
	shift
	;;
	-s-rp)
		smpk_info[rp]="yes"
	shift
	;;
	-s-ovrd)
		smpk_info[ovrd]="yes"
	shift
	;;
	--smek)
	if [[ "${smek_info[flag]}" == "no" ]]; then
		smek_info[flag]="yes"
		smek_info[file]="$2"
	else
		echo "ERR: SMEK provided multiple times"
		exit
	fi
    shift # past argument
    shift # past value
    ;;
	--smek-wp)
		smek_info[wp]="yes"
	shift
	;;
	--smek-rp)
		smek_info[rp]="yes"
	shift
	;;
	--smek-ovrd)
		smek_info[ovrd]="yes"
	shift
	;;
	--sr-bcfg)
	parse_validate_swrev_sec_brdcfg "$2"
	shift # past argument
    shift # past value
	;;
	--sr-bcfg-wp)
		swrev_sec_brdcfg_info[wp]="yes"
	shift
	;;
	--sr-bcfg-rp)
		swrev_sec_brdcfg_info[rp]="yes"
	shift
	;;
	--sr-bcfg-ovrd)
		swrev_sec_brdcfg_info[ovrd]="yes"
	shift
	;;
	--sr-sbl)
	parse_validate_swrev_sbl "$2"
	shift # past argument
    shift # past value
	;;
	--sr-sbl-wp)
		swrev_sbl_info[wp]="yes"
	shift
	;;
	--sr-sbl-rp)
		swrev_sbl_info[rp]="yes"
	shift
	;;
	--sr-sbl-ovrd)
		swrev_sbl_info[ovrd]="yes"
	shift
	;;
	--sr-sysfw)
	parse_validate_swrev_sysfw "$2"
	shift # past argument
    shift # past value
	;;
	--sr-sysfw-wp)
		swrev_sysfw_info[wp]="yes"
	shift
	;;
	--sr-sysfw-rp)
		swrev_sysfw_info[rp]="yes"
	shift
	;;
	--sr-sysfw-ovrd)
		swrev_sysfw_info[ovrd]="yes"
	shift
	;;
	-b|--bmpk)
	if [[ "${bmpk_info[flag]}" == "no" ]]; then
		bmpk_info[flag]="yes"
		bmpk_info[file]="$2"
	else
		echo "ERR: BMPK provided multiple times"
		exit
	fi
    shift # past argument
    shift # past value
    ;;
	-b-wp)
		bmpk_info[wp]="yes"
	shift
	;;
	-b-rp)
		bmpk_info[rp]="yes"
	shift
	;;
	-b-ovrd)
		bmpk_info[ovrd]="yes"
	shift
	;;
	--bmek)
	if [[ "${bmek_info[flag]}" == "no" ]]; then
		bmek_info[flag]="yes"
		bmek_info[file]="$2"
	else
		echo "ERR: BMEK provided multiple times"
		exit
	fi
    shift # past argument
    shift # past value
    ;;
	--bmek-wp)
		bmek_info[wp]="yes"
	shift
	;;
	--bmek-rp)
		bmek_info[rp]="yes"
	shift
	;;
	--bmek-ovrd)
		bmek_info[ovrd]="yes"
	shift
	;;
	-c|--cert)
	output_info[certificate]="$2"
	shift
	shift
	;;
	-h|--help)
	echo "$HELPTXT"
	exit
    shift # past argument
    shift # past value
    ;;
	-g|--gen)
	echo "# Generating dummy keys in keys/ folder"
	mkdir -p keys
	openssl rand 32 > keys/aes256.key
	openssl genrsa -out keys/smpk.pem 4096
	openssl genrsa -out keys/bmpk.pem 4096
	openssl rand 32 > keys/smek.key
	openssl rand 32 > keys/bmek.key
	exit
	shift # past argument
	shift # past value
	;;
	--msv)
	parse_validate_msv "$2"
	shift # past argument
	shift # past value
	;;
	--msv-wp)
		msv_info[wp]="yes"
	shift
	;;	
	--msv-rp)
		msv_info[rp]="yes"
	shift
	;;
	--msv-ovrd)
		msv_info[ovrd]="yes"
	shift
	;;
	--keycnt)
	parse_validate_key_cnt "$2"
	shift # past argument
	shift # past value
	;;
	--keycnt-wp)
		key_cnt_info[wp]="yes"
	shift
	;;	
	--keycnt-rp)
		key_cnt_info[rp]="yes"
	shift
	;;
	--keycnt-ovrd)
		key_cnt_info[ovrd]="yes"
	shift
	;;
	--keyrev)
		parse_validate_key_rev "$2"
	shift # past argument
	shift # past value
	;;
	--keyrev-wp)
		key_rev_info[wp]="yes"
	shift
	;;	
	--keyrev-rp)
		key_rev_info[rp]="yes"
	shift
	;;
	--keyrev-ovrd)
		key_rev_info[ovrd]="yes"
	shift
	;;
	--mpk-opt)
		parse_validate_mpk_opt "$2"
	shift
	shift
	;;
	--mek-opt)
		parse_validate_mek_opt "$2"
	shift
	shift
	;;

	*)
		echo "$1" is not a valid argument
		exit
	shift
	shift
	;;
esac
done

if [[ "${tifek_info[flag]}" == "no" ]]; then
	echo "ERR: TIFEK Public Key is required!!"
	exit
fi

# Check for presence of SM*K, BM*K (optional)
if [[ "${bmpk_info[flag]}" == "yes" && "${bmek_info[flag]}" == "yes" ]]; then
	secondary_cert_info[flag]="yes"
else
	secondary_cert_info[flag]="no"
fi

if [[ "${secondary_cert_info[flag]}" == "yes" ]]; then
	echo "Generating Dual signed certificate!!"
else
	echo "Generating Single signed certificate!!"
fi

mkdir -p tmpdir
mkdir -p configs
mkdir -p ../x509cert

if [[  ${smpk_info[flag]} == "no" ]]; then
	echo "INFO: Using random key(s) for signing certificate(s)"
	openssl genrsa -out tmpdir/smpk.pem 4096 2> /dev/null
	smpk_info[file]="tmpdir/smpk.pem"
fi
if [[  ${smek_info[flag]} == "no" ]]; then
	openssl rand 32 > tmpdir/smek.key
	smek_info[file]="tmpdir/smek.key"
fi
if [[  ${bmpk_info[flag]} == "no" ]]; then
	openssl genrsa -out tmpdir/bmpk.pem 4096 2> /dev/null
	bmpk_info[file]="tmpdir/bmpk.pem"
fi
if [[  ${bmek_info[flag]} == "no" ]]; then
	openssl rand 32 > tmpdir/bmek.key
	bmek_info[file]="tmpdir/bmek.key"
fi

# generate a random 256 bit key -- AES256 Key
if [ "${aes256key_info[flag]}" == "no" ]; then
	echo "GEN: AES256 key generated, since not provided"
	openssl rand 32 > tmpdir/aes256.key
	aes256key_info[file]="tmpdir/aes256.key"
fi

aes256key_info[val]=$(xxd -p -c 32 "${aes256key_info[file]}")

echo "" > "${output_info[hash_csv]}"

######################################################################################
# CRYPTOGRAPHIC OPERATIONS TO CALC HASH/ENC.
######################################################################################

echo "# encrypt aes256 key with tifek public part"
# encrypt <PUBLIC-KEY.PEM> <DATA> <ENCRYPTED-OUTPUT>
encrypt "${tifek_info[file]}" "${aes256key_info[file]}" tmpdir/enc_aes_key.enc

echo "# encrypt SMPK-priv signed aes256 key(hash) with tifek public part"
# sign_the_hash <PRIV-KEY.PEM> <OUTPUT> <INPUT>
sign_the_hash "${smpk_info[file]}" tmpdir/smpk_sign_aes256.sign "${aes256key_info[file]}"
# Block Size is 256 Bytes => 2048 bits
dd if=tmpdir/smpk_sign_aes256.sign of=tmpdir/smpk_sign_aes256_1.sign bs=256 count=1 status=none
dd if=tmpdir/smpk_sign_aes256.sign of=tmpdir/smpk_sign_aes256_2.sign bs=256 skip=1 count=1 status=none
# encrypt <PUBLIC-KEY.PEM> <DATA> <ENCRYPTED-OUTPUT>
encrypt "${tifek_info[file]}" tmpdir/smpk_sign_aes256_1.sign tmpdir/enc_smpk_signed_aes_key_1.enc
encrypt "${tifek_info[file]}" tmpdir/smpk_sign_aes256_2.sign tmpdir/enc_smpk_signed_aes_key_2.enc
cat tmpdir/enc_smpk_signed_aes_key_1.enc tmpdir/enc_smpk_signed_aes_key_2.enc > tmpdir/enc_smpk_signed_aes_key.enc

echo "# encrypt smpk-pub hash using aes256 key"
# gen_pub_key_x509_extension <PRIV.PEM> <PUB.DER> <PUB.HASH> <SMPKH/BMPKH> <IV> <RS> <FIELD> <OUTPUT>
gen_pub_key_x509_extension "${smpk_info[file]}" tmpdir/smpkpub.der tmpdir/smpkh "SMPKH" \
		tmpdir/smpkh.iv tmpdir/smpkh.rs tmpdir/smpkhfield tmpdir/aesenc_smpkh.enc

if [[  ${smek_info[flag]} == "yes" ]]; then
echo "# encrypt smek (sym key) using aes256 key"
fi
# gen_sym_key_x509_extension <SMEKH/BMEKH> <SYMKEY> <SYMKEY.HASH> <IV> <RS> <FIELD> <OUTPUT>
gen_sym_key_x509_extension "SMEKH" "${smek_info[file]}" tmpdir/smekh tmpdir/smek.iv \
		tmpdir/smek.rs tmpdir/smekfield tmpdir/aesenc_smek.enc

if [[ ${extotp_info[flag]} == "yes" ]]; then
echo "# encrypt ext_otp using aes256 key"
# gen_ext_otp_x509_extension <ext_otp_data> <HASH> <IV> <RS> <FIELD> <OUTPUT>
gen_ext_otp_x509_extension "${extotp_info[file]}" tmpdir/extotp_hash tmpdir/extotp.iv \
		tmpdir/extotp.rs tmpdir/extotpfield tmpdir/aesenc_extotp.enc
fi

if [[ "${secondary_cert_info[flag]}" == "yes" ]]; then
	echo "# encrypt BMPK-priv signed aes256 key(hash) with tifek public part"
	# sign_the_hash <PRIV-KEY.PEM> <OUTPUT> <INPUT>
	sign_the_hash "${bmpk_info[file]}" tmpdir/bmpk_sign_aes256.sign "${aes256key_info[file]}"
	dd if=tmpdir/bmpk_sign_aes256.sign of=tmpdir/bmpk_sign_aes256_1.sign bs=256 count=1 status=none
	dd if=tmpdir/bmpk_sign_aes256.sign of=tmpdir/bmpk_sign_aes256_2.sign bs=256 skip=1 count=1 status=none
	# encrypt <PUBLIC-KEY.PEM> <DATA> <ENCRYPTED-OUTPUT>
	encrypt "${tifek_info[file]}" tmpdir/bmpk_sign_aes256_1.sign tmpdir/enc_bmpk_signed_aes_key_1.enc
	encrypt "${tifek_info[file]}" tmpdir/bmpk_sign_aes256_2.sign tmpdir/enc_bmpk_signed_aes_key_2.enc
	cat tmpdir/enc_bmpk_signed_aes_key_1.enc tmpdir/enc_bmpk_signed_aes_key_2.enc > tmpdir/enc_bmpk_signed_aes_key.enc
	
	echo "# encrypt bmpk-pub hash using aes256 key"
	# gen_pub_key_x509_extension <PRIV.PEM> <PUB.DER> <PUB.HASH> <SMPKH/BMPKH> <IV> <RS> <FIELD> <OUTPUT>
	gen_pub_key_x509_extension "${bmpk_info[file]}" tmpdir/bmpkpub.der tmpdir/bmpkh "BMPKH" \
			tmpdir/bmpkh.iv tmpdir/bmpkh.rs tmpdir/bmpkhfield tmpdir/aesenc_bmpkh.enc
	
	echo "# encrypt bmek (sym key) using aes256 key"
	# gen_sym_key_x509_extension <SYMKEY> <SYMKEY.HASH> <IV> <RS> <FIELD> <OUTPUT>
	gen_sym_key_x509_extension "BMEKH" "${bmek_info[file]}" tmpdir/bmekh tmpdir/bmek.iv \
			tmpdir/bmek.rs tmpdir/bmekfield tmpdir/aesenc_bmek.enc
fi

######################################################################################
# POPULATE THE CONFIG FILES WITH HASH/ENC VALUES
######################################################################################
populate_config_primary

if [[ "${secondary_cert_info[flag]}" == "yes" ]]; then
	populate_config_secondary
else
	# Delete all entries corresponding to BMPK/BMEK from the config file
	sed -i "/.*[bB][mM].[kK].*/d" "${primary_cert_info[config]}"
fi

######################################################################################
# GENERATE x509 CERTIFICATE FROM CONFIG FILE(S)
######################################################################################

# create x509 certificate, signed with SMPK
openssl req -new -x509 -key "${smpk_info[file]}" \
		-nodes -outform der -out "${primary_cert_info[file]}" \
		-config "${primary_cert_info[config]}" \
		-sha512

if [[ "${secondary_cert_info[flag]}" == "yes" ]]; then
	# create x509 certificate, dual signed with BMPK
	SHAVAL=$(openssl dgst -sha512 -hex "${primary_cert_info[file]}" | sed -e "s/^.*= //g")
	SHALEN=$(wc -c < "${primary_cert_info[file]}")
	sed "s/SEC_CERT_SHA512/${SHAVAL}/" templates/config_bmpk_template.txt > "${secondary_cert_info[config]}"
	sed -i "s/SEC_CERT_LENGTH/${SHALEN}/" "${secondary_cert_info[config]}"
	openssl req -new -x509 -key "${bmpk_info[file]}" \
		-nodes -outform der -out "${secondary_cert_info[file]}" \
		-config "${secondary_cert_info[config]}" \
		-sha512

	# Concatenate the 2 certificates 
	cat "${secondary_cert_info[file]}" "${primary_cert_info[file]}" > "${output_info[certificate]}"
	du -b "${secondary_cert_info[file]}" "${primary_cert_info[file]}" "${output_info[certificate]}"
else
	cat "${primary_cert_info[file]}" > "${output_info[certificate]}"
	du -b "${primary_cert_info[file]}" "${output_info[certificate]}"
fi

# rm -rf tmpdir
echo "# SHA512 Hashes of keys are stored in ${output_info[hash_csv]} for reference.."