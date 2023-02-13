import argparse
import csv
import os
import time
import multiprocessing
from functools import partial


PARALLEL_PROCESSES = os.cpu_count()
saved_addr = set()
removed_addr = set()
def write_instr(instr_dir: str, instr):
    addr = instr[0]
    with open(os.path.join(instr_dir, addr), "a") as instr_file:
        writer = csv.writer(instr_file)
        writer.writerow(instr[2:])


def read_trace(trace_file: str, instr_dir: str, crash: bool):
    
    with open(trace_file, "r") as f:
        instr_list = list(csv.reader(f, delimiter=","))
        cur_addr = set()
        write_addr = list()
        for (i, instr) in enumerate(instr_list):
            if i == 0:
                continue
            if crash:
                instr.append(1)
            else:
                instr.append(0)
            addr = instr[0] + f"_{instr[1]}"
            if addr in removed_addr:
                continue
            if crash or addr in saved_addr:
                write_addr.append(instr)
                saved_addr.add(addr)
                cur_addr.add(addr)
    
        with multiprocessing.Pool(PARALLEL_PROCESSES) as pool:
            func = partial(write_instr, instr_dir)
            pool.map(func, write_addr)


        if crash:
            for addr in list(saved_addr):
                if addr not in cur_addr:
                    if os.path.exists(os.path.join(instr_dir, addr)):
                        os.system(f'rm -rf {os.path.join(instr_dir, addr)}')
                    saved_addr.remove(addr)
                    removed_addr.add(addr)
        


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str)
    parser.add_argument("-instr", type=str, default="instr")
    args = parser.parse_args()
    trace_dir = args.i
    instr_dir = args.instr
    print('creating instr dir')
    os.system(f'mkdir -p {instr_dir}')
    print('reading trace files')
    crashes_trace = os.path.join(trace_dir, "crashes")
    non_crashes_trace = os.path.join(trace_dir, "non_crashes")
    processed = 0
    print('processing crashes trace file')
    start50_time = time.time()
    for file in os.listdir(crashes_trace):
        if processed % 50 == 0:
            end50_time = time.time()
            print(f'processed: {processed}, use: {end50_time - start50_time}s')
            start50_time = time.time()
        read_trace(os.path.join(crashes_trace, file), instr_dir, True)
        processed = processed + 1
    end_time = time.time()

    processed = 0
    print('processing non_crashes trace file')
    start_time = time.time()
    start50_time = time.time()
    for file in os.listdir(non_crashes_trace):
        if processed % 50 == 0:
            end50_time = time.time()
            print(f'processed: {processed}, use: {end50_time - start50_time}s')
            start50_time = time.time()
        read_trace(os.path.join(non_crashes_trace, file), instr_dir, False)
        processed = processed + 1


    print(f'make instructions time: {end_time - start_time}s')

if __name__ == '__main__':
    main()
