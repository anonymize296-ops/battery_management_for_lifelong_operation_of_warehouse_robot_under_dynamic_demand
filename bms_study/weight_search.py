from pymoo.core.problem import Problem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize

import random
import os
import subprocess
import json
import csv
import math

import numpy as np


def create_a_scenario_file(lambdas: list[float]) -> str:
    scenario_file = f"scenario_1_goal_programming_w1_{lambdas[0]}_w2_{lambdas[1]}_w3_{lambdas[2]}_w4_{lambdas[3]}_w5_{lambdas[4]}"
    data = {
        "algorithm": {
            "algorithm_type": "goal_programming",
            "battery_min": 20,
            "n_thread": 16,
            "time_horizon": 4,
        },
        "battery_configuration": {},
        "charge_station": 6,
        "demand": {
            "demand_type": "CONST",
            "min_value": 40,
            "max_value": 40,
            "time_period": 39,
            "step": 3,
        },
        "dt": 3,
        "goal_programming_parameter": {
            "alpha_weights": lambdas,
            "obj1_work_mode_lower_bound": 30,
            "obj2_battery_lower_bound": 30,
            "obj3_battery_upper_bound": 70,
        },
        "robot_initial_battery": [
            44.0,
            32.0,
            46.0,
            44.0,
            52.0,
            39.0,
            41.0,
            40.0,
            39.0,
            46.0,
            52.0,
            30.0,
            38.0,
            48.0,
            37.0,
            43.0,
            51.0,
            34.0,
            43.0,
            54.0,
            50.0,
            31.0,
            38.0,
            43.0,
            54.0,
            33.0,
            43.0,
            37.0,
            54.0,
            49.0,
            46.0,
            37.0,
            54.0,
            35.0,
            51.0,
            44.0,
            40.0,
            54.0,
            48.0,
            53.0,
        ],
        "scenario": "./pareto_front/" + scenario_file,
        "time": 240,
    }

    scenario_file_loc = "./pareto_front/" + scenario_file + ".json"

    with open(scenario_file_loc, "w+") as json_file:
        json.dump(data, json_file, indent=4)

    return scenario_file_loc


class WeightSearch(Problem):

    def __init__(self):
        super().__init__(
            n_var=5,        # decision variables (weights)
            n_obj=2,        # number of objectives
            xl=0, xu=1      # bounds
        )

    def construct_and_solve(self, w1, w2, w3, w4, w5):
        create_a_scenario_file([w1, w2, w3, w4, w5])
        
        data = None
        output_file = f"w1_{w1}_w2_{w2}_w3_{w3}_w4_{w4}_w5_{w5}"
        with open("./pareto_front/scenario_1_goal_programming_"+ output_file + ".json", "r") as json_file:
            data = json.load(json_file)

        lambdas = data["goal_programming_parameter"]["alpha_weights"]
        
        if not os.path.exists("./pareto_front/scenario_1_goal_programming_"+ output_file + ".csv"):
            procees = subprocess.run(["python3", "bms.py", "-s", "./pareto_front/scenario_1_goal_programming_"+output_file+".json",])
            process = subprocess.run(["mv",
                    "./output/goal_programming_output_milp.csv",
                    "./pareto_front/goal_programming_output_"
                    + output_file
                    + ".csv",])
            outfile = "./pareto_front/goal_programming_output_" + output_file + ".csv"
            outfile2 = "./pareto_front/scenario_1_goal_programming_" + output_file + ".csv"

            data_processing = {
                "data_processing_type": "calc_consumed_energy_and_demand",
                "file_locations": [outfile, outfile2],}

            scenario_file_loc = "./pareto_front/data_processing/" + "output_" + output_file + ".json"
            
            with open(scenario_file_loc, "w+") as json_file:
                json.dump(data_processing, json_file, indent=4)

            process = subprocess.run(
                ["python3", "data_processor.py", "-dp", scenario_file_loc],
                capture_output=True,
                text=True,)

            result = process.stdout.splitlines()
            demand_difference = result[0]
            computation_time = result[1]
            consumed_energy = result[2]
            print(demand_difference, computation_time)

            with open("./pareto_front/vary_weights.csv", "a", newline="") as csvfile:
                writer = csv.writer(csvfile)
                value = [
                    str(lambdas[0]),
                    str(lambdas[1]),
                    str(lambdas[2]),
                    str(lambdas[3]),
                    str(lambdas[4]),
                    str(demand_difference),
                    str(computation_time),
                    str(consumed_energy),
                ]
                writer.writerow(value)

            return demand_difference, consumed_energy
        else:
            with open("./pareto_front/vary_weights.csv", "r", newline="") as csvfile:
                for line in csvfile.readlines():
                    data = [float(x) for x in line.strip().split(",")]
                    if math.isclose(data[0], w1) and math.isclose(data[1], w2) and math.isclose(data[2], w3) and math.isclose(data[3], w4) and math.isclose(data[4], w5):
                        return data[5], data[7]
            
        
    def _evaluate(self, X, out, *args, **kwargs):
        F = []

        for x in X:
            w1, w2, w3, w4, w5 =  x / (x.sum() + 1e-8) # To normalize weights

            print(w1, w2, w3, w4, w5)
            
            demand_difference, consumed_energy = self.construct_and_solve(w1, w2, w3, w4, w5)
            # extract results
            f1 = demand_difference
            f2 = consumed_energy

            F.append([f1, f2])

        out["F"] = np.array(F, dtype=float)



if __name__ == "__main__":
    problem = WeightSearch()
    
    algorithm = NSGA2(pop_size=20)

    res = minimize(problem,
               algorithm,
               ('n_gen', 50),
               seed=42,
               verbose=True)