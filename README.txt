minimal build+run instuctions


linux
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native" -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -flto"
cd build
ninja
./frc_sim --scene ./assets/scenes/test_floor_robot2.json --dt 0.01 --fps 90

windows
.\make-ninja.bat
cd build
ninja
.\frc_sim.exe --scene ./assets/scenes/test_floor_robot2.json --dt 0.01 --fps 90
