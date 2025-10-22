#import scipy

# TODO this file is in charge of using scipy to optimize our constants.
# It uses the datasets as ground truth for optimization

NUM_CONSTANTS = 9
constants = []

def parse_constants(filename: str):
    with open(filename, 'r') as f:
        for line in f:
            constraint = line.split(' ')[1]
            if constraint[0] == '[':
                # It is a soft parameter
                lo, hi = constraint.split(',')
                constants.append((float(lo[1:]), float(hi.strip()[:-1])))
            else:
                # It is a hard parameter
                constants.append(float(constraint))

parse_constants('parameter_constraints.txt')

print(constants)
