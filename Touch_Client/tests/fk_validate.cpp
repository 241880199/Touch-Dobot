// fk_validate — standalone C++ FK validation
// Reads CSV input, runs Kinematics FK, writes JSON output
// Build: build_fk_validate.bat
// Run: fk_validate.exe input.csv cpp_output.json

#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>

#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"

// ===== Minimal CSV parser for: label,j1,j2,j3,j4,j5,j6 =====

struct Config {
    std::string label;
    double joints[6];
};

static std::vector<Config> parseCSV(const char* path) {
    std::vector<Config> configs;
    FILE* f = fopen(path, "r");
    if (!f) {
        printf("ERROR: cannot open %s\n", path);
        return configs;
    }
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        // Skip empty lines and comments
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

        Config c;
        char label[128] = {0};
        int n = sscanf(line, "%127[^,],%lf,%lf,%lf,%lf,%lf,%lf",
                       label,
                       &c.joints[0], &c.joints[1], &c.joints[2],
                       &c.joints[3], &c.joints[4], &c.joints[5]);
        if (n == 7) {
            c.label = label;
            configs.push_back(c);
        }
    }
    fclose(f);
    return configs;
}

// ===== Minimal JSON writer =====

static void writeJSON(const char* path, const std::vector<Config>& inputs) {
    FILE* f = fopen(path, "w");
    if (!f) {
        printf("ERROR: cannot write %s\n", path);
        return;
    }

    fprintf(f, "{\n  \"configs\": [\n");

    for (size_t i = 0; i < inputs.size(); i++) {
        const auto& cfg = inputs[i];

        // Run FK
        Vec3 positions[7];
        Kinematics::computeJointPositions(cfg.joints, positions);

        double T[4][4];
        Kinematics::composeTransform(cfg.joints, T);
        Vec3 ee_from_T = {T[0][3], T[1][3], T[2][3]};

        // Write JSON entry
        fprintf(f, "    {\n");
        fprintf(f, "      \"label\": \"%s\",\n", cfg.label.c_str());
        fprintf(f, "      \"input\": [%.10g, %.10g, %.10g, %.10g, %.10g, %.10g],\n",
                cfg.joints[0], cfg.joints[1], cfg.joints[2],
                cfg.joints[3], cfg.joints[4], cfg.joints[5]);

        // joint_positions
        fprintf(f, "      \"joint_positions\": [\n");
        for (int j = 0; j < 7; j++) {
            fprintf(f, "        [%.15g, %.15g, %.15g]%s\n",
                    positions[j].x, positions[j].y, positions[j].z,
                    j < 6 ? "," : "");
        }
        fprintf(f, "      ],\n");

        // ee_position
        fprintf(f, "      \"ee_position\": [%.15g, %.15g, %.15g],\n",
                positions[6].x, positions[6].y, positions[6].z);

        // ee_from_transform
        fprintf(f, "      \"ee_from_transform\": [%.15g, %.15g, %.15g]\n",
                ee_from_T.x, ee_from_T.y, ee_from_T.z);

        fprintf(f, "    }%s\n", i < inputs.size() - 1 ? "," : "");
    }

    fprintf(f, "  ]\n}\n");
    fclose(f);
    printf("fk_validate: %zu configs → %s\n", inputs.size(), path);
}

// ===== main =====

int main(int argc, char* argv[]) {
    const char* inputPath  = (argc > 1) ? argv[1] : "input.csv";
    const char* outputPath = (argc > 2) ? argv[2] : "cpp_output.json";

    auto configs = parseCSV(inputPath);
    if (configs.empty()) {
        printf("ERROR: no configs parsed from %s\n", inputPath);
        return 1;
    }

    writeJSON(outputPath, configs);
    return 0;
}
