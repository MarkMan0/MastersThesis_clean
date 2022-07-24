/**
 * @file fuzzy_system.h
 * @brief Contains .fis file contents from Matlab
 */
#pragma once

#include <string>

namespace fuzzy_systems {

  /**
   * @brief line controller fuzzy system designed in Matlab
   *
   */
  static inline const std::string line_control_fuzzy =
      R"fuzzy([System]
Name='Untitled2'
Type='mamdani'
Version=2.0
NumInputs=2
NumOutputs=1
NumRules=9
AndMethod='min'
OrMethod='max'
ImpMethod='min'
AggMethod='max'
DefuzzMethod='centroid'

[Input1]
Name='sensor_pos'
Range=[-1 1]
NumMFs=3
MF1='left':'trimf',[-1.83 -1 0]
MF2='center':'trimf',[-0.8333 0 0.8333]
MF3='right':'trimf',[0 1 1.84]

[Input2]
Name='dist_from_path'
Range=[-1 1]
NumMFs=3
MF1='left_far':'trimf',[-1.833 -1 -0.1667]
MF2='close':'trimf',[-0.828008994708995 0.00529100529100535 0.838591005291005]
MF3='right_far':'trimf',[0.1667 1 1.833]

[Output1]
Name='omega'
Range=[-1 1]
NumMFs=5
MF1='hard_left':'trimf',[-1.5 -1 -0.5]
MF2='straight':'trimf',[-0.5 0 0.5]
MF3='hard_right':'trimf',[0.5 1 1.5]
MF4='slight_left':'trimf',[-1 -0.5 0]
MF5='slight_right':'trimf',[0 0.5 1]

[Rules]
2 2, 2 (1) : 1
2 1, 2 (1) : 1
2 3, 2 (1) : 1
1 2, 4 (1) : 1
2 3, 4 (1) : 1
3 2, 5 (1) : 1
2 1, 5 (1) : 1
1 3, 1 (1) : 1
3 1, 3 (1) : 1
)fuzzy";

  static inline const std::string fuzzy_dual_senzor =
      R"fuzzy(
[System]
Name='dual_sensor'
Type='mamdani'
Version=2.0
NumInputs=2
NumOutputs=1
NumRules=9
AndMethod='min'
OrMethod='max'
ImpMethod='min'
AggMethod='max'
DefuzzMethod='centroid'

[Input1]
Name='sensor1'
Range=[-1 1]
NumMFs=3
MF1='left':'trimf',[-1.833 -1 -0.1667]
MF2='center':'trimf',[-0.8333 -2.776e-17 0.8333]
MF3='right':'trimf',[0.1667 1 1.833]

[Input2]
Name='sensor2'
Range=[-1 1]
NumMFs=3
MF1='left':'trimf',[-1.833 -1 -0.1667]
MF2='center':'trimf',[-0.8333 0 0.8333]
MF3='right':'trimf',[0.1667 1 1.833]

[Output1]
Name='output'
Range=[-1 1]
NumMFs=5
MF1='hard_left':'trimf',[-1.5 -1 -0.5]
MF2='left':'trimf',[-1 -0.5 0]
MF3='straight':'trimf',[-0.5 0 0.5]
MF4='right':'trimf',[0 0.5 1]
MF5='hard_right':'trimf',[0.5 1 1.5]

[Rules]
1 1, 1 (1) : 1
1 2, 4 (1) : 1
1 3, 5 (1) : 1
2 1, 2 (1) : 1
2 2, 3 (1) : 1
2 3, 4 (1) : 1
3 1, 1 (1) : 1
3 2, 2 (1) : 1
3 3, 5 (1) : 1

  )fuzzy";

}  // namespace fuzzy_systems
