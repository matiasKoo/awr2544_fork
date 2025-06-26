import argparse
import configparser

#------------------------------------------------
# This script requires two arguments
# First argument is the name of the .ini file containing the desired configuration
# Second argument is the name of the .h file created by the script
# for example:
#
# python cfg_init.py myconfig.ini myheader.h
#------------------------------------------------

parser = argparse.ArgumentParser()
parser.add_argument("cfg_fname", help="name of .ini file containing cfg")
parser.add_argument("out_fname", help="name of .h file created as output")
args = parser.parse_args()

config = configparser.ConfigParser()
config.read(args.cfg_fname)

profile_dict = {s:dict(config.items(s)) for s in config.sections()}
profile_dict = profile_dict["profileCfg"]
print(profile_dict)

profile_key_list = list(profile_dict.keys())

def comment_func(x):
    match x:
        case 'startfreqconst':
            ret_str = "// " + profile_dict[x] + " * 53.644 Hz = " + str(float(profile_dict[x]) * 53.644) + " Hz"
            return(ret_str)
        case 'idletimeconst':
            ret_str = "// " + profile_dict[x] + " * 10 ns = " + str(float(profile_dict[x]) * 10) + " ns = " + str(float(profile_dict[x])/100) + " us"
            return(ret_str)
        case 'adcstarttimeconst':
            ret_str = "// " + profile_dict[x] + " * 10 ns = " + str(float(profile_dict[x]) * 10) + " ns = " + str(float(profile_dict[x])/100) + " us"
            return(ret_str)
        case 'rampendtime':
            ret_str = "// " + profile_dict[x] + " * 10 ns = " + str(float(profile_dict[x]) * 10) + " ns = " + str(float(profile_dict[x])/100) + " us"
            return(ret_str)
        case 'freqslopeconst':
            ret_str = "// " + profile_dict[x] + " * 48.729 kHz/us = " + str(float(profile_dict[x]) * 48.729) + " kHz/us = " + str(float(profile_dict[x]) * 48.729 / 1000) + " MHz/us"
            return(ret_str)
        case 'digoutsamplerate':
            ret_str = "// " + profile_dict[x] + " * 1000 samples/s = " + str(float(profile_dict[x]) * 1000) + " Hz sample rate"
            return(ret_str)
        case _:
            return("")


file_string = ""

for i in range(len(profile_key_list)):
    comment_string = ""
    comment_string = comment_func(profile_key_list[i])

    substr = "CFG_PROFILE_" + profile_key_list[i] + " " + profile_dict[profile_key_list[i]] + "U "
    substr = substr.upper()
    substr = substr + comment_string + "\n"
    file_string = file_string + "#define " + substr

outname_string = args.out_fname
outname_string = outname_string[:-2]
outname_string = outname_string.upper()

file_string = "#ifndef "+ outname_string +"_H\n#define "+outname_string+"_H\n\n"+file_string+"\n#endif"

with open(args.out_fname,'w') as headerfile:
    headerfile.write(file_string)
