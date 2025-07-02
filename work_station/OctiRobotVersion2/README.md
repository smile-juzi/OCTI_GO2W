# unitree_sdk2
OCTI Robot

### Prebuild environment
* OS  (Ubuntu 20.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 9.4.0) 

### Installation
```bash
sudo ./install.sh

```

### Build examples
```bash
mkdir build
cd build
cmake ..
make
```

### Notice
The main cpp is in the directionary of main_control


### update
add api to kill thread ladder and vision etc..
    add mutex