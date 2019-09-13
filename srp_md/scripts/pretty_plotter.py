#!/usr/bin/env python
# In project imports


# Python imports
import csv
import argparse
import itertools
import numpy as np
import matplotlib.pyplot as plt


def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(description='Plot the results')

    # Main arguments
    parser.add_argument('--file', help='Specify the directory of file',
                        default="/home/jihwangk/Downloads/test.csv", type=str)

    args = parser.parse_args()
    return args


def main():
    # Get arguments
    args = parse_args()

    # Initialize lists of independent and dependent variables
    input_order = ['Time Stamp', 'Goal Type', 'Object #s', 'Demo #s', 'Goal #s', 'Noise %', 'Correct Demo #s',
                   'Factor Learner', 'Factors', 'Min Object #s', 'Max Object #s', 'Consistency', 'Commonsense']
    input_independent = ['Goal Type', 'Object #s', 'Factor Learner', 'Factors', 'Consistency']
    num_run = 10

    # Initialize dictionaries
    category = {var: [] for var in input_independent}  # includes set of values that independent variables can take
    data_dict = {}  # includes the average correct demo for each run

    # Read in the file
    with open(args.file) as csvfile:
        data = csv.reader(csvfile, delimiter=',')

        for row in data:
            # Initialize key for data_dict
            key_list = []

            for var in input_independent:
                # Fill in the key with values of independent variables
                ele = row[input_order.index(var)]
                key_list.append(ele)

                # Fill in the categories
                if ele not in category[var]:
                    category[var].append(ele)

            key_list = tuple(key_list)

            # Fill in data_dict and data_count
            num_demo = int(row[input_order.index('Demo #s')])
            num_correct = int(row[input_order.index('Correct Demo #s')])
            if key_list in data_dict.keys():
                if num_demo not in data_dict[key_list][0]:
                    data_dict[key_list][0].append(num_demo)
                    data_dict[key_list][1].append(num_correct)
                else:
                    ind = data_dict[key_list][0].index(num_demo)
                    data_dict[key_list][1][ind] += num_correct
            else:
                data_dict[key_list] = [[num_demo], [num_correct]]

        # Average the entries in data_dict by value in data_count
        for key_ls in data_dict.keys():
            data_dict[key_ls][1] = [correct_bef / float(num_run) for correct_bef in data_dict[key_ls][1]]

    # Now plot the graphs!
    # For each independent variables, do
    for i in range(len(input_independent)):
        var = input_independent[i]
        # For each value it can take, plot and compare how other variables change the result
        for val in category[var]:
            # For each entry in data_dict:
            for key_list in data_dict.keys():
                if key_list[i] == val:
                    plt.plot(data_dict[key_list][0], data_dict[key_list][1],
                             marker='o', linewidth=2, linestyle='dashed', label=key_list)

            plt.title(var + ": " + val, fontsize=14)
            plt.xlabel("Number of Demos", fontsize=14)
            plt.xticks(fontsize=14)
            plt.ylabel("Number of Correct Goals", fontsize=14)
            plt.yticks(fontsize=14)
            plt.legend()
            plt.show()
            plt.clf()

    for key_list in data_dict.keys():
        plt.plot(data_dict[key_list][0], data_dict[key_list][1],
                 marker='o', linewidth=2, linestyle='dashed', label=key_list)

    plt.title("EVERYTHING", fontsize=14)
    plt.xlabel("Number of Demos", fontsize=14)
    plt.xticks(fontsize=14)
    plt.ylabel("Number of Correct Goals", fontsize=14)
    plt.yticks(fontsize=14)
    plt.legend()
    plt.show()
    plt.clf()


if __name__ == '__main__':
    main()

rng = np.arange(50)
rnd = np.random.randint(0, 10, size=(3, rng.size))
yrs = 1950 + rng

fig, ax = plt.subplots(figsize=(5, 3))
ax.stackplot(yrs, rng + rnd, labels=['Eastasia', 'Eurasia', 'Oceania'])
ax.set_title('Combined debt growth over time')
ax.legend(loc='upper left')
ax.set_ylabel('Total debt')
ax.set_xlim(xmin=yrs[0], xmax=yrs[-1])
fig.tight_layout()
