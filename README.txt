minimal build+run instuctions
required components: vcpkg, cmake, C++ compiler, python + robotpy pillow opencv numpy tk, ffmpeg
i reccomend using ninja-build


linux
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native" -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -flto"
cd build
ninja
./frc_sim --scene ./assets/scenes/955.json --robot ./assets/bodies/robot.json@127.0.0.1:5810 --dt 0.01 --fps 90


windows
.\make-ninja.bat
cd build
ninja
.\frc_sim --scene ./assets/scenes/955.json --robot ./assets/bodies/robot.json@127.0.0.1:5810 --dt 0.01 --fps 90


src/controller.py can be used as a basic robot-code replacement
src/multiplayer-dashboard.py is a GUI version


streaming (multiplayer)
./frc_sim --scene ./assets/scenes/955.json --robot ./assets/bodies/robot.json@127.0.0.1:5810 --dt 0.01 --fps 60 --stream 127.0.0.1:5000 --stream-fps 20
python3 src/multiplayer-dashboard.py
