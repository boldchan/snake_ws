import numpy as np
from Neuron import input_neuron, hidden_neuron, output_neuron, special_neuron
import parameters as p
from os import path, getcwd


def load_data(filename='target_data.txt'):
    input_data = []
    output_data = []
    with open(filename, 'r') as f:
        for line in f:
            data = list(map(float, line[:-1].split(',')))
            data[:2] /= np.sqrt(data[0]**2 + data[1]**2)
            data_rel = [0, 0, 0]
            if data[1] > 0:
                data_rel[0] = data[1]
            else:
                data_rel[2] = -data[1]
            if (data[0] < 0):
                data_rel[1] = -data[0]
            input_data.append(data_rel)
            output_data.append(data[2:])
    return {'input': input_data, 'output': output_data}


def slice_data(data, i, j):
    return {key: data[key][i:j + 1] for key in data.keys()}


class Two_Layer_SNN(object):
    '''
    a two-layer spiking neuron network with a IF-Neuron without leakage as input layer, two LIF-Neuron 
    as hidden and output layer

    The architecture should be input layer - affine - hidden layer - affine - output layer, in which 
    synapse applies first alpha function then affine and its output is the postsynaptic potential (PSP) 
    '''
    @staticmethod
    def __generate_weight(input_dim, hidden_dim, output_dim):
        n = input_dim * hidden_dim
        ratio = 0.8
        W1 = np.array(
            [2 + 2 * np.random.randn() for x in range(int(ratio * n))] +
            [(-1 - np.random.randn()) for x in range(n - int(ratio * n))]
        )
        np.random.shuffle(W1)
        W1 = W1.reshape((input_dim, hidden_dim))

        n = hidden_dim * output_dim
        ratio = 0.8
        W2 = np.array(
            [2 + 2 * np.random.randn() for x in range(int(ratio * n))] +
            [(-1 - np.random.randn()) for x in range(n - int(ratio * n))]
        )
        np.random.shuffle(W2)
        W2 = W2.reshape((hidden_dim, output_dim))
        W3 = np.random.randn(input_dim, 1)
        return W1, W2, W3

    @staticmethod
    def cal_degree(pos):
        act_l = pos[0] / 5
        act_r = pos[1] / 5
        deg = [-180 * act_l, 180 * act_r]
        return deg

    def __init__(self, input_dim=3, hidden_dim=3, output_dim=2, T=p.T, dt=p.dt, t_rest=0, load=False):
        self.T = T  # total time to simulate(ms)
        self.dt = dt  # simulation time step(ms)
        self.t_rest = t_rest  # initial refrectory time

        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim

        self.input_neuron = input_neuron(input_dim)
        self.hidden_neuron = hidden_neuron(hidden_dim)
        self.output_neuron = output_neuron(output_dim)
        self.special_neuron = special_neuron(1)

        if load:
            self.load_weight()
        else:
            self.W1, self.W2, self.W3 = Two_Layer_SNN.__generate_weight(
                input_dim, hidden_dim, output_dim
            )

        self.STDP1 = np.zeros_like(self.W1)
        self.STDP2 = np.zeros_like(self.W2)
        self.STDP3 = np.zeros_like(self.W3)

        self.reward1 = np.zeros_like(self.W1)
        self.reward2 = np.zeros_like(self.W2)
        self.reward3 = np.zeros_like(self.W3)

        self.eta = p.eta_max  # learning rate, ignore decay for now
        self.error = 0

        self.STDP1 = np.zeros((input_dim, hidden_dim))
        self.STDP2 = np.zeros((hidden_dim, output_dim))

    def save_weight(self):
        np.savetxt('W1.txt', self.W1)
        np.savetxt('W2.txt', self.W2)
        np.savetxt('W3.txt', self.W3)

    def load_weight(self):
        self.W1 = np.loadtxt(
            path.join(getcwd(), 'src/snake_control/scripts/W1.txt'))
        self.W2 = np.loadtxt(
            path.join(getcwd(), 'src/snake_control/scripts/W2.txt'))
        self.W3 = np.loadtxt(
            path.join(getcwd(), 'src/snake_control/scripts/W3.txt'))

    def update_stdp(self, l1, l2):
        '''
        l1: former layer output(spike)
        l2: latter layer output(spike)
        based on paper "Two-trace model for spike-timing-dependent synaptic plasticity"
        '''
        shape = l1.shape[0], l2.shape[0]
        apre = np.zeros(shape)
        apost = np.zeros(shape)
        stdp = np.zeros(shape)
        for t in range(int(self.T / self.dt) - 1):
            for i in range(shape[0]):
                for j in range(shape[1]):
                    apre[i][j] -= apre[i][j] / p.taupre * self.dt
                    apost[i][j] -= apost[i][j] / p.taupost * self.dt
                    if(l1[i][t]):
                        apre[i][j] += p.Apre
                        stdp[i][j] += apost[i][j]
                    if(l2[j][t]):
                        apost[i][j] += p.Apost
                        stdp[i][j] += apre[i][j]
        return stdp

    def updateET(self):  # Eligibility trace
        def g(w):
            return 1 - c1 * w * np.exp((-1 * c2 / wmax) * abs(w))
        c1 = p.c1
        c2 = p.c2
        wmax = p.wmax
        g1 = g(self.W1)
        g2 = g(self.W2)
        g3 = g(self.W3)
        return g1, g2, g3

    # Function called at the end of each iteration
    # Caution - reward, STDP, g1 must have same size as W
    def calculate_deltaW(self):
        g1, g2, g3 = self.updateET()  # Update the eligliblity trace first

        deltaW1 = self.eta * self.reward1 * self.STDP1 * g1
        self.W1 = np.clip(self.W1 + deltaW1, -p.wmax, p.wmax)

        deltaW2 = self.eta * self.reward2 * self.STDP2 * g2
        self.W2 = np.clip(self.W2 + deltaW2, -p.wmax, p.wmax)

        deltaW3 = self.eta * self.reward3 * self.STDP3 * g3
        self.W3 = np.clip(self.W3 + deltaW3, -p.wmax, p.wmax)

    def update_rewards(self, out, expected):
        '''
        out is the output of snn, expected is the expected value
        '''
        # turn right
        # pdb.set_trace()
        reward_irrev = p.reward_irrev
        if np.abs(expected[1]) < np.abs(expected[0]):
            self.error += np.abs(expected[1] - out[1])
            # turn right
            rewardR = (np.abs(expected[1]) - np.abs(out[1]))/p.ymax
            # rewardL = 0
            if rewardR > 0:
                rewardL = reward_irrev
            elif np.abs(out[1]) > np.abs(out[0]):
                rewardL = reward_irrev
            else:
                rewardL = -reward_irrev
            if np.abs(out[1]) > 90 and np.abs(out[1]) < np.abs(out[0]):
                rewardL = rewardR = 0
        else:
            self.error += np.abs(expected[0] - out[0])
            # turn left
            rewardL = (np.abs(expected[0]) - np.abs(out[0]))/p.ymax
            # rewardR = 0
            if rewardL > 0:
                rewardR = reward_irrev
            elif np.abs(out[0]) > np.abs(out[1]):
                rewardR = reward_irrev
            else:
                rewardR = -reward_irrev
            if np.abs(out[0]) > 90 and np.abs(out[0]) < np.abs(out[1]):
                rewardL = rewardR = 0

        self.reward2[:, 0] = rewardL
        self.reward2[:, 1] = rewardR

        for i in range(self.hidden_dim):  # this can be made compact
            reward = (abs(self.W2[i][0])*rewardL + abs(self.W2[i][1])
                      * rewardR) / (abs(self.W2[i][0]) + abs(self.W2[i][1]))
            self.reward1[:, i] = reward

    def __feed(self, data, alpha):
        _, out1 = self.input_neuron.forward(data)
        _, out2 = self.hidden_neuron.forward(out1, self.W1)
        _, out3 = self.output_neuron.decode(out2, self.W2)
        res = Two_Layer_SNN.cal_degree([out3[0][-1], out3[1][-1]])
        self.update_rewards(res, alpha)
        self.STDP1 = self.update_stdp(out1, out2)
        self.STDP2 = self.update_stdp(out2, out3)
        self.calculate_deltaW()
        return res

    def test(self, data):
        _, out1 = self.input_neuron.forward(data)
        _, out2 = self.hidden_neuron.forward(out1, self.W1)
        _, out3 = self.output_neuron.decode(out2, self.W2)
        res = Two_Layer_SNN.cal_degree([out3[0][-1], out3[1][-1]])
        return res

    def train(self, data, eta_reduction=None):
        num_data = len(data['input'])
        self.error = 0
        if not eta_reduction:
            eta_reduction = (p.eta_max - p.eta_min) / num_data
        for d, alpha in zip(data['input'], data['output']):
            output = self.__feed(d, alpha)
            print('Predicted :    {}'.format(output))
            print('Actual Value:  {}'.format(alpha))
            print('Learning rate: {}'.format(self.eta))
        self.error = self.error / num_data
        self.eta = self.eta - eta_reduction

    def simulate(self, data, iterations):
        input_data = data['input']
        alpha = data['output']
        for i in range(iterations):
            output = self.__feed(input_data, alpha)
        return output


if __name__ == '__main__':
    snn = Two_Layer_SNN(hidden_dim=30, load=False)

    data = load_data()
    iterations = 5
    eta = (p.eta_max - p.eta_min) / iterations
    error = []
    for t in range(iterations):
        # train only data number 12
        # snn.train(slice_data(data, 12, 12), eta)
        snn.train(data, eta)
        error.append(snn.error)
        print ('Average angle error in each iteration : {}'.format(error))
        res = snn.test([np.sqrt(2)/2, 0, 0])
        print(res)
        # snn.train(data, eta)
        # snn.save_weight()
