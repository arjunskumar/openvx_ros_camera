# openvx_ros_camera

<details> 
  <summary> Install OpenVX 1.3 on Ubuntu 18.04 </summary>

```
cd openvx_ws

git clone --recursive https://github.com/KhronosGroup/OpenVX-sample-impl.git

cd OpenVX-sample-impl/

python Build.py --os=Linux --arch=64 --conf=Debug --conf_vision --enh_vision --conf_nn

export OPENVX_DIR=$(pwd)/install/Linux/x64/Debug

export VX_TEST_DATA_PATH=$(pwd)/cts/test_data/

mkdir build-cts

cd build-cts

cmake -DOPENVX_INCLUDES=$OPENVX_DIR/include -DOPENVX_LIBRARIES=$OPENVX_DIR/bin/libopenvx.so\;$OPENVX_DIR/bin/libvxu.so\;pthread\;dl\;m\;rt -DOPENVX_CONFORMANCE_VISION=ON -DOPENVX_USE_ENHANCED_VISION=ON -DOPENVX_CONFORMANCE_NEURAL_NETWORKS=ON ../cts/

cmake --build .

LD_LIBRARY_PATH=./lib ./bin/vx_test_conformance

```

</details>


<details>
  <summary> Make Changes in CMakeLists.txt </summary>
  [L21](https://github.com/arjunskumar/openvx_ros_camera/blob/9cd82caea1390605d346b4537079307d73bfdc2b/CMakeLists.txt#L21)

  [L22](https://github.com/arjunskumar/openvx_ros_camera/blob/9cd82caea1390605d346b4537079307d73bfdc2b/CMakeLists.txt#L22)

</details>


