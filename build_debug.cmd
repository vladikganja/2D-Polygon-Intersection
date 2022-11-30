mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=TRUE -DBUILD_GMOCK=OFF ..
cmake --build . --config Debug
Pause
