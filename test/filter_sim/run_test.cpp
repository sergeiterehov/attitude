/**
 * Тестовый раннер: читает sim_data.csv, прогоняет через фильтр, выводит results.csv.
 *
 * Использование:
 *   ./run_test --filter mahony < sim_data.csv > results_mahony.csv
 *   ./run_test --filter custom < sim_data.csv > results_custom.csv
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "filter_backend.h"
#include "mahony_backend.h"
#include "complementary_backend.h"

#define MAX_LINE 1024

void print_usage(const char* prog) {
  fprintf(stderr, "Usage: %s --filter <mahony|complementary>\n", prog);
  fprintf(stderr, "  Reads sim_data.csv from stdin, writes results.csv to stdout\n");
}

int main(int argc, char** argv) {
  // Parse arguments
  const char* filter_name = nullptr;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--filter") == 0 && i + 1 < argc) {
      filter_name = argv[i + 1];
      i++;
    } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_usage(argv[0]);
      return 0;
    }
  }

  if (!filter_name) {
    print_usage(argv[0]);
    return 1;
  }

  // Create filter backend
  FilterBackend* filter = nullptr;
  if (strcmp(filter_name, "mahony") == 0) {
    filter = new MahonyBackend();
  } else if (strcmp(filter_name, "complementary") == 0) {
    filter = new ComplementaryBackend();
  } else {
    fprintf(stderr, "Unknown filter: %s\n", filter_name);
    return 1;
  }

  filter->begin(100.0f);  // 100 Hz

  // Print header
  printf("scenario,t_ms,true_roll,true_pitch,true_yaw,v_ground,v_vertical,"
         "ax,ay,az,gx,gy,gz,"
         "filter_roll,filter_pitch,error_roll,error_pitch\n");

  // Read CSV from stdin
  char line[MAX_LINE];
  bool first_line = true;
  float dt = 0.01f;  // 100 Hz

  while (fgets(line, sizeof(line), stdin)) {
    // Skip header
    if (first_line) {
      first_line = false;
      if (strncmp(line, "scenario,", 9) == 0) {
        continue;
      }
    }

    // Parse CSV line
    char scenario[64];
    int t_ms;
    float true_roll, true_pitch, true_yaw;
    float v_ground, v_vertical;
    float ax, ay, az;
    float gx, gy, gz;

    int parsed = sscanf(line, "%63[^,],%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", scenario, &t_ms,
                        &true_roll, &true_pitch, &true_yaw, &v_ground, &v_vertical, &ax, &ay, &az,
                        &gx, &gy, &gz);

    if (parsed != 13) {
      fprintf(stderr, "Warning: failed to parse line: %s", line);
      continue;
    }

    // Run filter
    FilterOutput out = filter->update(gx, gy, gz, ax, ay, az, dt);

    // Compute errors (degrees)
    float error_roll = out.roll * 57.29578f - true_roll;
    float error_pitch = out.pitch * 57.29578f - true_pitch;

    // Output
    printf("%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f\n",
           scenario, t_ms, true_roll, true_pitch, true_yaw, v_ground, v_vertical, ax, ay, az, gx,
           gy, gz, out.roll * 57.29578f, out.pitch * 57.29578f, error_roll, error_pitch);
  }

  delete filter;
  return 0;
}
