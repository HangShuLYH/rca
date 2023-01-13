import argparse
import csv
import os

import numpy as np

from instruction import *
InstrSet = InstructionSet()


def read_trace(trace_file: str, crash: bool):
    with open(trace_file, "r") as f:
        instr_list = list(csv.reader(f, delimiter=","))
        cur_list = set()
        for (i, instr) in enumerate(instr_list):
            if i == 0:
                continue
            addr = instr[0]
            InstrSet.add_instruction(addr, instr[2:])
            cur_list.add(addr)
        if not crash:
            for addr in list(InstrSet.instr_map):
                if addr not in cur_list:
                    InstrSet.instr_map.pop(addr)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str)
    args = parser.parse_args()
    trace_dir = args.i
    print('reading trace files')
    crashes_trace = os.path.join(trace_dir, "crashes")
    non_crashes_trace = os.path.join(trace_dir, "non_crashes")
    for file in os.listdir(crashes_trace):
        read_trace(os.path.join(crashes_trace, file), True)
    for file in os.listdir(non_crashes_trace):
        read_trace(os.path.join(non_crashes_trace, file), False)
    print('visualizing_pca')
    InstrSet.visualize_pca()


if __name__ == '__main__':
    main()
