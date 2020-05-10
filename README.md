# construction-robotics

sai2-examples depends on [sai2-interfaces](https://github.com/manips-sai-org/sai2-interfaces).
Clone it and ensure that its definitions are exported.

To build the examples:
```
mkdir build && cd build
cmake ..
make
```

Binaries will placed in the `bin/` folder.

To fully start this example, you can use the launch script provided:
```
./launch.sh
```

After that's all done, navigate to `localhost:8000` to interact with the example.
