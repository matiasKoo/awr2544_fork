import argparse
import configparser

#------------------------------------------------
# This script requires two arguments
# First argument is the name of the .ini file containing the desired configuration
# Second argument is the name of the .h file created by the script
# for example:
#
# python cfg_init.py myconfig.ini myheader.h
#
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

file_string = ""

for i in range(len(profile_key_list)):
    substr = "CFG_PROFILE_" + profile_key_list[i] + " " + profile_dict[profile_key_list[i]] + "U\n"
    substr = substr.upper()
    file_string = file_string + "#define " + substr

outname_s = args.out_fname
outname_s = outname_s[:-2]
outname_s = outname_s.upper()

file_string = "#ifndef "+ outname_s +"_H\n#define "+outname_s+"_H\n\n"+file_string+"\n#endif"

with open(args.out_fname,'w') as headerfile:
    headerfile.write(file_string)
