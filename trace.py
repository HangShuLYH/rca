import os
import argparse
import subprocess
import multiprocessing
import random
import time
from functools import partial
PIN_EXE = './pin/pin'
PIN_TOOL = './pin/source/tools/tracerTool/obj-intel64/tracer.so'
TMP_PATH = '/tmp/rca'
PARALLEL_PROCESSES = os.cpu_count()


def trace_file(exe: str, input_dir, file):
    out_file = os.path.join(TMP_PATH, file)
    input_file = os.path.join(input_dir, file)
    cmd = f"{PIN_EXE} -t {PIN_TOOL} -o {out_file} -- {exe} < {input_file}"
    try:
        subprocess.run(cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as err:
        pass


def trace_all(input_dir: str, output_dir: str, exe: str):
    crash_input = os.path.join(input_dir, "crashes")
    non_crash_input = os.path.join(input_dir, "non_crashes")

    print('creating tmp dirs')
    os.system('mkdir -p ' + os.path.join(TMP_PATH, 'non_crashes'))
    os.system('mkdir -p ' + os.path.join(TMP_PATH, 'crashes'))
    files = []
    files.extend([f"crashes/{x}" for x in os.listdir(crash_input)])
    files.extend([f"non_crashes/{x}" for x in os.listdir(non_crash_input)])
    random.SystemRandom().shuffle(files)
    print('tracing files')
    start_time = time.time()
    with multiprocessing.Pool(PARALLEL_PROCESSES) as pool:
        func = partial(trace_file, exe, input_dir)
        pool.map(func, files)
    trace_time = time.time()
    print(f'trace files time: {trace_time - start_time}s')
    print('moving tmp to output')
    if os.path.exists(output_dir):
        os.system(f"rm -rf {output_dir}")
    os.system(f"mkdir -p {output_dir}")
    cmd = f"mv {TMP_PATH}/* {output_dir}"
    os.system(cmd)
    print('deleting tmp')
    cmd = f"rm -rf {TMP_PATH}"
    os.system(cmd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', type=str)
    parser.add_argument('-o', type=str)
    parser.add_argument('-e', type=str)
    args = parser.parse_args()
    input_path = args.i
    output_path = args.o
    exe_path = args.e
    trace_all(input_path, output_path, exe_path)


if __name__ == '__main__':
    main()
