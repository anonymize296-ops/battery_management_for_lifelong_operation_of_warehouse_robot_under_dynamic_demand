import matplotlib.pyplot as plt
import pandas as pd
from DemandSignal import DemandSignal, DemandType
import numpy as np

def main():
    # battery_data = pd.read_csv('./scenario_2_bcs_trapezoidal_achievable.csv')
    # battery_data = pd.read_csv('./output/scenario_1_milp/comparison/Threshold25/scenario_1_threshold_25_nc_9.csv')
    #battery_data = pd.read_csv('./scenarios/comment/scenario_1_goal_programming_nc_6_30_70.csv')
    battery_data = pd.read_csv('./scenario_1_goal_programming_nc_6.csv')
    # battery_data = pd.read_csv('./scenario_1_goal_programming_nc_6_30_70.csv')
    # battery_data = pd.read_csv('./scenarios/comment/scenario_1_rule_based_nc_6_25_80.csv')
    #battery_data = pd.read_csv('./scenarios/comment/scenario_1_rule_based_nc_6_30_70.csv')
    # battery_data = pd.read_csv('./output/scenario_1_milp/comparison/Threshold40/ChargeStation/scenario_1_threshold_40_nc_6.csv')
    print(battery_data)
    for i in range(40):
        plt.plot(battery_data[battery_data['rid'] == i]['time'],battery_data[battery_data['rid'] == i]['battery'], marker='*')

    # data = pd.read_csv('./output/goal_programming_output_bcs_.csv')

    # plt.plot(data['demand_actual'])    
    # demand = DemandSignal(DemandType.TRIANGULAR_STEP, 34, 37, 39, 3)
    # data = []
    # for i in np.linspace(3, 120, 40):
    #     data.append(demand.get_demand_value(i))
    # plt.plot(np.linspace(3, 120, 40), data)
    plt.show()


if __name__ == "__main__":
    main()
