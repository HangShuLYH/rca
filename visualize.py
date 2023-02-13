import os
import argparse
from sklearn.decomposition import PCA
from sklearn.feature_selection import VarianceThreshold
from sklearn.preprocessing import StandardScaler
from sklearn.manifold import TSNE
import umap
import matplotlib.pyplot as plt
import multiprocessing
from functools import partial
import time
import csv
import numpy as np


PARALLEL_PROCESSES = os.cpu_count()
def visualize(instr_dir: str, output_dir: str, algorithm, file_name: str):
    with open(os.path.join(instr_dir, file_name), "r") as f:
        data_lable = np.array(list(csv.reader(f, delimiter=",")))
        data = StandardScaler().fit_transform(data_lable[:, :-1])
        label = data_lable[:, -1]
        try:
            data = VarianceThreshold(threshold=0.0).fit_transform(data)
        except ValueError:
            return
        
        plt.subplots(dpi=80, figsize=(9, 6))
        index = (label == np.ones_like(label))
        if data.shape[1] == 1:
            plt.scatter(data[:, 0][index], np.ones_like(data)[index], label = 'crash', c = 'b')
            plt.scatter(data[:, 0][~index], np.ones_like(data)[~index], label = 'non_crash', c= 'r')
        else:
            vis = algorithm(n_components=2)
            results = vis.fit_transform(data)
            x1 = results[:, 0]
            x2 = results[:, 1]
            plt.scatter(x1[index], x2[index], label = 'crash', c = 'b')
            plt.scatter(x1[~index], x2[~index], label = 'non_crash', c = 'r')
        plt.legend()
        plt.savefig(os.path.join(output_dir, file_name))
        plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', type=str, default='./instr')
    parser.add_argument('-o', type=str, default='./visualize')
    parser.add_argument('-method', type=str, default='pca', choices=['pca', 'tsne', 'umap'])
    args = parser.parse_args()
    instr_dir = args.i
    output_dir = args.o + "_" + args.method
    method = args.method
    method_list = {'pca': PCA, 'tsne': TSNE, 'umap': umap}
    print('creating visualize_dir')
    os.system(f'mkdir -p {output_dir}')
    print('visualizing pictures')
    work_list = list()
    for file_name in os.listdir(instr_dir):
        work_list.append(file_name)
    start_time = time.time()
    with multiprocessing.Pool(PARALLEL_PROCESSES) as pool:
        func = partial(visualize, instr_dir, output_dir, method_list[method])
        pool.map(func, work_list)
    end_time = time.time()
    print(f'visualize use: {end_time - start_time}s')

if __name__ == '__main__':
    main()