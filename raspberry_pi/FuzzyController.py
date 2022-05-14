import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

error_distance = ctrl.Antecedent(np.arange(-320, 321, 1), 'error_distance')
dutyCycle = ctrl.Consequent(np.arange(0, 101, 1), 'dutyCycle')

error_distance['negativeHigh'] = fuzz.trapmf(error_distance.universe, [-320, -320, -192, -64])
error_distance['negativeLow'] = fuzz.trimf(error_distance.universe, [-192, -64, 0])
error_distance['zero'] = fuzz.trimf(error_distance.universe, [-64, 0, 64])
error_distance['positiveLow'] = fuzz.trimf(error_distance.universe, [0, 64, 192])
error_distance['positiveHigh'] = fuzz.trapmf(error_distance.universe, [64, 192, 320, 320])

dutyCycle['Low'] = fuzz.trapmf(dutyCycle.universe, [0, 0, 50, 70])
dutyCycle['Medium'] = fuzz.trimf(dutyCycle.universe, [50, 70, 80])
dutyCycle['High'] = fuzz.trapmf(dutyCycle.universe, [70, 80, 100, 100])

rule1 = ctrl.Rule(error_distance['negativeHigh'], dutyCycle['High'])
rule2 = ctrl.Rule(error_distance['negativeLow'], dutyCycle['Medium'])
rule3 = ctrl.Rule(error_distance['zero'], dutyCycle['Low'])
rule4 = ctrl.Rule(error_distance['positiveLow'], dutyCycle['Medium'])
rule5 = ctrl.Rule(error_distance['positiveHigh'], dutyCycle['High'])
rotation_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
rotation = ctrl.ControlSystemSimulation(rotation_ctrl)

distanceToBox = ctrl.Antecedent(np.arange(0, 101, 1), 'distanceToBox')  # distance in cm
dutyCycleForward = ctrl.Consequent(np.arange(0, 101, 1), 'dutyCycleForward')

distanceToBox['Close'] = fuzz.trapmf(distanceToBox.universe, [0, 0, 10, 20])
distanceToBox['Average'] = fuzz.trapmf(distanceToBox.universe, [10, 20, 40, 50])
distanceToBox['Far'] = fuzz.trapmf(distanceToBox.universe, [40, 50, 100, 100])

rule1F = ctrl.Rule(distanceToBox['Close'], dutyCycle['Low'])
rule2F = ctrl.Rule(distanceToBox['Average'], dutyCycle['Medium'])
rule3F = ctrl.Rule(distanceToBox['Far'], dutyCycle['High'])

forward_ctrl = ctrl.ControlSystem([rule1F, rule2F, rule3F])
forward = ctrl.ControlSystemSimulation(forward_ctrl)

def rotation_controller(e):
    rotation.input['error_distance'] = e
    rotation.compute()
    return rotation.output['dutyCycle']


def forward_controller(d):
    forward.input['distanceToBox'] = d
    forward.compute()
    return forward.output['dutyCycle']
