import sys


def print_flak(file_name: str):
    flag_exporting = False
    flak_file = open(file_name, 'r')
    flak_lines = flak_file.readlines()
    flak_file.close()

    print("# flak: " + file_name)
    for flak_line in flak_lines:
        if flak_line.startswith("# flak export"):
            flag_exporting = True
        elif flak_line.startswith("# flak end"):
            flag_exporting = False
        elif flag_exporting:
            print(flak_line.rstrip())

    print("# flak end: " + file_name)


source = open(sys.argv[1], 'r')
prefix = "# flak emplacement "

lines = source.readlines()
source.close()
flag_omitted = False
for line in lines:
    if line.startswith("# flak emplacement "):
        line = line[len("# flak emplacement "):].rstrip()
        file_name = line + ".py"
        print_flak(file_name)
    elif line.startswith("# flak omit"):
        flag_omitted = True
    elif line.startswith("# flak unomit"):
        flag_omitted = False
    else:
        print(line.rstrip())
