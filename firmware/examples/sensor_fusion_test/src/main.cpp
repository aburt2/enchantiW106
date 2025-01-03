/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 2024 Muhammad Haziq
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <zephyr/kernel.h>
#include <errno.h>

// Sensor fusion
#include "imu_orientation.h"
#include <cmath>
#include <iostream>
#include <unistd.h>

void benchmarkSensorFusion(int num_loops) {
  IMU_Orientation sensor_fusion;
  int count = num_loops;
  uint32_t start, end, elapsed_time;
  float time_per_loop;
  start = k_cycle_get_32();
  while (count--) {
      sensor_fusion.setAccelerometerValues(0, 1, 0);
      sensor_fusion.setGyroscopeRadianValues(0, 0, 0, 0.001);
      sensor_fusion.setMagnetometerValues(0, 1, 2);
      sensor_fusion.update();
  }
  end = k_cycle_get_32();
  elapsed_time = end-start;
  time_per_loop = float(elapsed_time) / float(num_loops);
  elapsed_time = k_cyc_to_ms_ceil32(elapsed_time);


  // Print time
  std::cout << "Iterations: " << num_loops << " total_time: " << elapsed_time << "ms"<< " unit_time: " << time_per_loop << "us"<< std::endl;
}

/********* MAIN **********/
int main(void)
{
    // Initialise devices
    std::cout << "Benchmark RW612 Sensor Fusion Performance " << std::endl;
    int trials = 5;
    int i = 0;

    while(trials--) {
        std::cout << "Start Test " << i << std::endl;
        i++;
        benchmarkSensorFusion(100000);
    }
}
