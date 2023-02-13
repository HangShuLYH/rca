import argparse
import re
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', type=str)
    parser.add_argument('-o', type=str)
    args = parser.parse_args()
    input_file = args.i
    output_file = args.o
    rankings = dict()
    with open(input_file, "r") as fi:
        line = fi.readline()
        while line:
            s = [float(s) for s in re.findall(r'-?\d+\.?\d*', line)]
            rankings[str(s[0])] = (s[1],s[2],s[3],s[4],s[5])
            line = fi.readline()
    rankings = sorted(rankings.items(), key=lambda x:(-x[1][0] + x[1][1] + x[1][2] + x[1][3]), reverse=True)
    with open(output_file, "w") as f:
        for (instr, (crash_dis, non_crash_dis, crash_dis2, non_crash_dis2, non_crash_data_num)) in rankings:
            f.write(f'{instr}: {crash_dis}, {non_crash_dis}, {crash_dis2}, {non_crash_dis2}, {non_crash_data_num}, {-crash_dis + non_crash_dis + crash_dis2 + non_crash_dis2}\n')
if __name__ == '__main__':
    main()