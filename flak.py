import sys


def print_flak(file_name: str):
    flag_exporting = False
    flak_file = open(file_name, 'r')
    flak_lines = flak_file.readlines()
    flak_file.close()

    print("#flak: " + file_name)
    for flak_line in flak_lines:
        if flak_line.startswith("#flak export"):
            flag_exporting = True
        elif flak_line.startswith("#flak end"):
            flag_exporting = False
        elif flag_exporting:
            print(flak_line.rstrip())

    print("#end flak: " + file_name)


source = open(sys.argv[1], 'r')
prefix = "#flak emplacement "

lines = source.readlines()
source.close()
for line in lines:
    if not line.startswith(prefix):
        print(line.rstrip())
        continue

    line = line[len(prefix):].rstrip()
    file_name = line + ".py"
    print_flak(file_name)
