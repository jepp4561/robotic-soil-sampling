
## Prerequisites
* docker

## Initialization
After cloning this project, if it was not pulled using --recursive-submodules then the submodules can be initialized using:
```bash
git submodule update --init --recursive
```
To compile custom libraries for the pico, you can add the path to the compile.sh script and run it:
```bash
./compile.sh
```