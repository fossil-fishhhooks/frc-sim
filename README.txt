minimal build+run instuctions
required components: vcpkg, cmake, C++ compiler, python + robotpy
i reccomend using ninja-build


linux
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native" -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -flto"
cd build
ninja
./frc_sim --scene ./assets/scenes/955.json --dt 0.01 --fps 90

windows
.\make-ninja.bat
cd build
ninja
.\frc_sim.exe --scene ./assets/scenes/955.json --dt 0.01 --fps 90


src/controller.py can be used as a basic robot-code replacement
