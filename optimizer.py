from scipy.optimize import minimize
from slipless_model import dynamics
import numpy as np
import pandas as pd


# TODO this file is in charge of using scipy to optimize our constants.
# It uses the datasets as ground truth for optimization

NUM_CONSTANTS = 9
constants = []

#initial vector guess
x0 = []

#parse trace csv file
df = pd.read_csv("datasets/trace1.csv")

#parse cols
INPUT_COLS = ['x', 'y', 'v', 'yaw', 'swangle', 'throttle', 'timestep']
TARGET_COLS = ['nextx', 'nexty', 'nextv', 'nextyaw']

model_data_inputs = df[INPUT_COLS].values
data_ground_truth = df[TARGET_COLS].values

print(f"model_data_inputs shape: {model_data_inputs.shape}")
print(f"data_ground_truth shape: {data_ground_truth.shape}")


def parse_constants(filename: str):
    with open(filename, 'r') as f:
        for line in f:
            constraint = line.split(' ')[1]
            if constraint[0] == '[':
                # It is a soft parameter
                lo, hi = constraint.split(',')
                lo_float = float(lo[1:])
                hi_float = float(hi.strip()[:-1])
                constants.append((lo_float, hi_float))
                x0.append(np.mean([lo_float, hi_float]))
            else:
                # It is a hard parameter
                constants.append((float(constraint), float(constraint)))
                x0.append(float(constraint))

parse_constants('parameter_constraints.txt')
print(constants)

#f(c, input, output)-->Error : Objective function for optimizer
def objective_function(c, X_input, Y_target):

    N = X_input.shape[0]
    P = Y_target.shape[1]
    Y_predicted = np.empty_like(Y_target)

    for i in range(N):
        current_state = X_input[i, [0, 1, 3, 2]].tolist()

        action = X_input[i, 4:6].tolist();
        timestep = X_input[i, 6].tolist();
        
        next_state = np.empty(P).tolist();
    
        dynamics(current_state, action, next_state, timestep, c)

        Y_predicted[i, 0] = next_state[0] # nextx
        Y_predicted[i, 1] = next_state[1] # nexty
        Y_predicted[i, 2] = next_state[3] # nextv (speed)
        Y_predicted[i, 3] = next_state[2] # nextyaw (yaw)
    
    #compute the error
    residuals = Y_predicted - Y_target
    return np.sum(residuals**2)


#Optimizer
result = minimize(
    fun=objective_function,
    x0=x0,
    args=(model_data_inputs, data_ground_truth), # Pass your data to the objective function
    method='L-BFGS-B',                            
    bounds=constants,                                # Apply the soft and fixed bounds
    options={'disp': True, 'ftol': 1e-6}
)

print("Optimized Constants:", result.x)