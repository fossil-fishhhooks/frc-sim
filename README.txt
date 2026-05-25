cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native" -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -flto"
cd build
ninja

