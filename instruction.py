import os

from sklearn.decomposition import PCA
import matplotlib.pyplot as plt


class InstructionData:
    def __init__(self):
        self.data = list()

    def add_data(self, data):
        self.data.append(data)


class InstructionSet:
    def __init__(self):
        self.instr_map = dict()

    def add_instruction(self, addr: str, data):
        if addr not in self.instr_map.keys():
            self.instr_map[addr] = InstructionData()
        self.instr_map[addr].add_data(data)

    def visualize_pca(self):
        save_dir = "visualize"
        if not os.path.exists(save_dir):
            os.system(f'mkdir {save_dir}')
        for addr in self.instr_map.keys():
            pca = PCA(n_components=2)
            pca.fit(self.instr_map[addr].data)
            result = pca.transform(self.instr_map[addr].data)
            fig, ax = plt.subplots(dpi=80, figsize=(9, 6))
            pca_x1 = result[:, 0]
            pca_x2 = result[:, 1]
            plt.scatter(pca_x1, pca_x2)
            # plt.show()
            save_name = os.path.join(save_dir, addr)
            plt.savefig(save_name)

    def rank(self):
        pass
