import argparse
import os
import csv
from sklearn.preprocessing import StandardScaler
from sklearn.feature_selection import VarianceThreshold
import numpy as np
import time
from sklearn.metrics.pairwise import cosine_similarity

def euler_dis(data, center):
    return np.sqrt(np.sum(np.square(data - center), axis = 1)).mean()

def cos_dis(data, center):
    return cosine_similarity(data, center.reshape(1,-1)).mean()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', type=str, default='./instr2')
    parser.add_argument('-o', type=str, default='./rankings.out')
    args = parser.parse_args()
    instr_dir = args.i
    out_file = args.o
    print('Calculating distance')
    start_time = time.time()
    rankings = dict()
    for file_name in os.listdir(instr_dir):
        with open(os.path.join(instr_dir, file_name), "r") as f:
            data_lable = np.array(list(csv.reader(f, delimiter=",")))
            data = StandardScaler().fit_transform(data_lable[:, :-1])
            try:
                data = VarianceThreshold(threshold=0.0).fit_transform(data)
            except ValueError:
                continue
            
            label = data_lable[:, -1]
            index = (label == np.ones_like(label))
            crash_data = data[index]
            non_crash_data = data[~index]
            if non_crash_data.shape[0] < 1000:
                continue
            center = np.mean(crash_data, axis = 0)
            crash_data_var = crash_data.var(axis=0).mean()
            non_crash_data_var = non_crash_data.var(axis=0).mean()
            crash_data_std = crash_data.std(axis=0).mean()
            non_crash_data_std = non_crash_data.std(axis=0).mean()
            crash_dis = euler_dis(crash_data, center)
            non_crash_dis = euler_dis(non_crash_data, center)

            center2 = np.mean(non_crash_data, axis = 0)
            crash_dis2 = euler_dis(crash_data, center2)
            non_crash_dis2 = euler_dis(non_crash_data, center2)
            rankings[file_name] = (crash_dis, non_crash_dis, crash_dis2, non_crash_dis2, non_crash_data.shape[0])
            # print(rankings[file_name])
            # print(crash_data_var)
            # print(non_crash_data_var)
    # value = list()
    # instrs = list()
    # for (instr, (crash_dis, non_crash_dis, crash_dis2, non_crash_dis2, non_crash_data_num)) in rankings.items():
    #     instrs.append(instr)
    #     value.append([crash_dis, non_crash_dis, crash_dis2, non_crash_dis2, non_crash_data_num])
    # value = np.array(value)
    # value = StandardScaler().fit_transform(value)
    # rankings = {instrs[i]: tuple(value[i,:]) for i in range(value.shape[0])}
    rankings = sorted(rankings.items(), key=lambda x:(-x[1][0] + x[1][1] + x[1][2] + x[1][3]), reverse=True)
    with open(out_file, "w") as f:
        for (instr, (crash_dis, non_crash_dis, crash_dis2, non_crash_dis2, non_crash_data_num)) in rankings:
            f.write(f'{instr}: {crash_dis}, {non_crash_dis}, {crash_dis2}, {non_crash_dis2}, {non_crash_data_num}, {-crash_dis + non_crash_dis + crash_dis2 + non_crash_dis2}\n')
    end_time = time.time()
    print(f'use: {end_time - start_time}s')
    


if __name__ == '__main__':
    main()